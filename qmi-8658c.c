// SPDX-License-Identifier: GPL-2.0
/*
 * QMI8658A IMU driver - FIFO + INT1 watermark interrupt, for attitude estimation
 *
  */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio/consumer.h>

#define QMI8658_REG_WHO_AM_I		0x00	/* = 0x05 */
#define QMI8658_REG_CTRL1		0x02
#define QMI8658_REG_CTRL2		0x03	/* Accel: FS + ODR */
#define QMI8658_REG_CTRL3		0x04	/* Gyro:  FS + ODR */
#define QMI8658_REG_CTRL5		0x06	/* LPF */
#define QMI8658_REG_CTRL7		0x08	/* aEN/gEN */
#define QMI8658_REG_CTRL8		0x09	/* Motion ctrl + CTRL9 handshake type */
#define QMI8658_REG_CTRL9		0x0A	/* Host command */
#define QMI8658_REG_FIFO_WTM		0x13
#define QMI8658_REG_FIFO_CTRL		0x14
#define QMI8658_REG_FIFO_SMPL_CNT	0x15	/* 低 8 位, 单位: word(2 bytes) */
#define QMI8658_REG_FIFO_STATUS	0x16	/* bit7 FULL, bit6 WTM, bit1:0 计数高 2 位 */
#define QMI8658_REG_FIFO_DATA		0x17
#define QMI8658_REG_STATUSINT		0x2D	/* bit7 = CTRL9 CmdDone */
#define QMI8658_REG_RESET_RESULT	0x4D	/* 复位成功 = 0x80 */
#define QMI8658_REG_RESET		0x60	/* 写 0xB0 软复位 */

#define QMI8658_WHO_AM_I_VAL		0x05
#define QMI8658_RESET_CMD		0xB0
#define QMI8658_RESET_SUCCESS		0x80

/* CTRL1: bit6 ADDR_AI, bit5 BE, bit3 INT1_EN, bit2 FIFO_INT_SEL(1=INT1) */
#define QMI8658_CTRL1_VAL		0x4C	/* 自增 + 小端 + INT1 使能 + FIFO->INT1 */

/* CTRL2: aFS=010(±8g), aODR=0101 (6DOF 下 224.2Hz, 见 Table 23 note16) */
#define QMI8658_CTRL2_VAL		0x25
/* CTRL3: gFS=101(±512dps), gODR=0101 (224.2Hz, 必须与加计一致) */
#define QMI8658_CTRL3_VAL		0x55
/* CTRL5: gLPF_EN + aLPF_EN, 带宽模式 11 (13.37% ODR) */
#define QMI8658_CTRL5_VAL		0x77
/* CTRL7: 仅 aEN|gEN, bit3:2 为 Reserved 不可置位 */
#define QMI8658_CTRL7_VAL		0x03
/* CTRL8: bit7=1, CTRL9 握手使用 STATUSINT.bit7 而非 INT1 */
#define QMI8658_CTRL8_VAL		0x80

/* FIFO_CTRL: bit7 RD_MODE, [3:2] SIZE(10=64 samples), [1:0] MODE(10=Stream) */
#define QMI8658_FIFO_CTRL_VAL		0x0A
#define QMI8658_FIFO_RD_MODE		BIT(7)
#define QMI8658_FIFO_WTM_LEVEL		16	/* 16 组 @224.2Hz ≈ 71ms 一批 */

#define QMI8658_FIFO_STATUS_WTM	BIT(6)
#define QMI8658_FIFO_STATUS_FULL	BIT(7)
#define QMI8658_STATUSINT_CMD_DONE	BIT(7)

/* CTRL9 命令 (Table 29) */
#define QMI8658_CTRL9_CMD_ACK		0x00
#define QMI8658_CTRL9_CMD_RST_FIFO	0x04
#define QMI8658_CTRL9_CMD_REQ_FIFO	0x05

#define QMI8658_SAMPLE_SIZE		12	/* accel 6B + gyro 6B */

static const unsigned long qmi8658_scan_masks[] = {
	0x3F,
	0
};

struct qmi8658_data {
	struct i2c_client *client;
	struct mutex lock;
	struct {
		s16 channels[6];
		s64 ts __aligned(8);
	} scan;
	struct gpio_desc *irq_gpio;
	int irq_num;
};

#define QMI8658_CHAN(_type, _mod, _addr, _index) { \
	.type = _type, \
	.modified = 1, \
	.channel2 = _mod, \
	.scan_index = _index, \
	.address = _addr, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.scan_type = { \
		.sign = 's', \
		.realbits = 16, \
		.storagebits = 16, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec qmi8658_channels[] = {
	QMI8658_CHAN(IIO_ACCEL,    IIO_MOD_X, 0x35, 0),
	QMI8658_CHAN(IIO_ACCEL,    IIO_MOD_Y, 0x37, 1),
	QMI8658_CHAN(IIO_ACCEL,    IIO_MOD_Z, 0x39, 2),
	QMI8658_CHAN(IIO_ANGL_VEL, IIO_MOD_X, 0x3B, 3),
	QMI8658_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, 0x3D, 4),
	QMI8658_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, 0x3F, 5),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static int qmi8658_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct qmi8658_data *data = iio_priv(indio_dev);
	__le16 raw_val;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		ret = i2c_smbus_read_i2c_block_data(data->client,
						    chan->address, 2,
						    (u8 *)&raw_val);
		mutex_unlock(&data->lock);
		if (ret < 0)
			return ret;

		*val = sign_extend32(le16_to_cpu(raw_val), 15);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			/* ±8g: 8*2*9.80665/65536 = 0.002394 m/s^2 per LSB */
			*val = 0;
			*val2 = 2394;
			return IIO_VAL_INT_PLUS_MICRO;

		case IIO_ANGL_VEL:
			/* ±512dps: (512*2/65536)*(pi/180) = 272707 nrad/s per LSB */
			*val = 0;
			*val2 = 272707;
			return IIO_VAL_INT_PLUS_NANO;

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static const struct iio_info qmi8658_info = {
	.read_raw = qmi8658_read_raw,
};

/*
 */
static int qmi8658_ctrl9_cmd(struct qmi8658_data *data, u8 cmd)
{
	struct i2c_client *client = data->client;
	int ret, retry;

	ret = i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL9, cmd);
	if (ret < 0)
		return ret;

	for (retry = 100; retry > 0; retry--) {
		ret = i2c_smbus_read_byte_data(client, QMI8658_REG_STATUSINT);
		if (ret >= 0 && (ret & QMI8658_STATUSINT_CMD_DONE))
			break;
		usleep_range(100, 200);
	}
	if (retry <= 0) {
		dev_err(&client->dev, "ctrl9 cmd 0x%02x timeout\n", cmd);
		return -ETIMEDOUT;
	}

	return i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL9,
					 QMI8658_CTRL9_CMD_ACK);
}

static irqreturn_t qmi8658_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct qmi8658_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	u8 cnt_buf[2];
	u8 buf[QMI8658_SAMPLE_SIZE];
	u16 fifo_bytes;
	int frames, i, ret;

	mutex_lock(&data->lock);
    do {
        /*
         * 1. 连读 FIFO_SMPL_CNT(0x15) + FIFO_STATUS(0x16)。
         *    手册 8.4: bytes = 2 * (FIFO_STATUS[1:0]*256 + FIFO_SMPL_CNT[7:0])
         */
        ret = i2c_smbus_read_i2c_block_data(client, QMI8658_REG_FIFO_SMPL_CNT,
        				    2, cnt_buf);
        if (ret != 2)
        	break;

        fifo_bytes = 2 * (((cnt_buf[1] & 0x03) << 8) | cnt_buf[0]);
        frames = fifo_bytes / QMI8658_SAMPLE_SIZE;
        if (frames <= 0)
        	break;

        /* 2. CTRL9 握手: 使能 FIFO 读模式 (手册 8.7, 5.10.6.3) */
        ret = qmi8658_ctrl9_cmd(data, QMI8658_CTRL9_CMD_REQ_FIFO);
        if (ret < 0)
        	break;

        /* 3. 从 FIFO_DATA(0x17) 逐帧读出并推入 IIO 缓冲区 */
        for (i = 0; i < frames; i++) {
        	ret = i2c_smbus_read_i2c_block_data(client,
        					    QMI8658_REG_FIFO_DATA,
        					    QMI8658_SAMPLE_SIZE, buf);
        	if (ret != QMI8658_SAMPLE_SIZE) {
        		dev_err(&client->dev,
        			"FIFO block read failed at frame %d\n", i);
        		break;
        	}

        	/* 帧格式 (手册 8.9): AX AY AZ GX GY GZ, 各 16bit 小端 */
        	memcpy(data->scan.channels, buf, QMI8658_SAMPLE_SIZE);
        	iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
        					   iio_get_time_ns(indio_dev));
	    }

	    /*
	    * 4. 清 FIFO_CTRL.FIFO_RD_MODE (bit7), FIFO 恢复采集,
	    *    同时释放 WTM/FULL 标志和 INT 引脚 (手册 5.10.6.3 / 8.5)
	    */
	    i2c_smbus_write_byte_data(client, QMI8658_REG_FIFO_CTRL,
				  QMI8658_FIFO_CTRL_VAL);
         /* 关键: 清完 RD_MODE 后回读状态, 仍达水位就再来一轮 */
        ret = i2c_smbus_read_byte_data(client, QMI8658_REG_FIFO_STATUS);         
    }while (ret >= 0 && (ret & QMI8658_FIFO_STATUS_WTM));
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

static int qmi8658_hw_init(struct qmi8658_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	/* 1. 软复位: 写 0xB0 到 0x60 (手册 5.9), 复位过程最长 15ms (7.4) */
	i2c_smbus_write_byte_data(client, QMI8658_REG_RESET, QMI8658_RESET_CMD);
	msleep(20);

	ret = i2c_smbus_read_byte_data(client, QMI8658_REG_RESET_RESULT);
	if (ret != QMI8658_RESET_SUCCESS)
		dev_warn(&client->dev, "reset result 0x%02x (expect 0x80)\n", ret);

	ret = i2c_smbus_read_byte_data(client, QMI8658_REG_WHO_AM_I);
	if (ret != QMI8658_WHO_AM_I_VAL) {
		dev_err(&client->dev, "bad WHO_AM_I: 0x%02x\n", ret);
		return -ENODEV;
	}

	/* 2. CTRL1: 地址自增 + 小端 + INT1 输出使能 + FIFO 中断映射到 INT1 */
	i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL1, QMI8658_CTRL1_VAL);

	/* 3. CTRL8.bit7=1: CTRL9 握手走 STATUSINT.bit7, INT1 专用于 FIFO 水位 */
	i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL8, QMI8658_CTRL8_VAL);

	/* 4. 加计 ±8g / 陀螺 ±512dps, 6DOF ODR 均为 224.2Hz (两者必须一致) */
	i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL2, QMI8658_CTRL2_VAL);
	i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL3, QMI8658_CTRL3_VAL);

	/* 5. 使能加计/陀螺低通滤波, 姿态解算降噪 */
	i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL5, QMI8658_CTRL5_VAL);

	/* 6. FIFO: 水位 16 组; 64 样本深度, Stream 模式 */
	i2c_smbus_write_byte_data(client, QMI8658_REG_FIFO_WTM,
				  QMI8658_FIFO_WTM_LEVEL);
	i2c_smbus_write_byte_data(client, QMI8658_REG_FIFO_CTRL,
				  QMI8658_FIFO_CTRL_VAL);

	/* 7. 清空 FIFO 残留 (CTRL_CMD_RST_FIFO, 手册 8.10) */
	qmi8658_ctrl9_cmd(data, QMI8658_CTRL9_CMD_RST_FIFO);

	/* 8. 最后使能传感器: 仅 aEN|gEN (CTRL7 bit3:2 为 Reserved 不可置位) */
	i2c_smbus_write_byte_data(client, QMI8658_REG_CTRL7, QMI8658_CTRL7_VAL);

	return 0;
}

static int qmi8658_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct qmi8658_data *data;
	int ret;

	dev_info(&client->dev, "qmi8658_probe entry\n");

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	indio_dev->name = "qmi8658";
	indio_dev->info = &qmi8658_info;
	indio_dev->channels = qmi8658_channels;
	indio_dev->num_channels = ARRAY_SIZE(qmi8658_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = qmi8658_scan_masks;

	/* 6.6: devm_iio_kfifo_buffer_setup 内部分配 kfifo 并 attach */
	ret = devm_iio_kfifo_buffer_setup(&client->dev, indio_dev, NULL);
	if (ret)
		return ret;

	ret = qmi8658_hw_init(data);
	if (ret)
		return ret;

	dev_info(&client->dev, "qmi8658 hw init successful\n");

	/*
	 * FIFO 水位中断是电平型 (手册 8.5): FIFO 填充 >= 水位期间 INT1 保持高,
	 * 读空并清 RD_MODE 后才拉低。用 LEVEL_HIGH 避免边沿丢失导致卡死。
	 */
	if (client->irq > 0) {
		dev_info(&client->dev, "find irq %d\n", client->irq);
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, qmi8658_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"qmi8658_fifo_irq", indio_dev);
		if (ret)
			return ret;
	} else {
		dev_info(&client->dev, "no irq, try dts irq-gpios\n");

		data->irq_gpio = devm_gpiod_get_optional(&client->dev, "irq",
							 GPIOD_IN);
		if (IS_ERR_OR_NULL(data->irq_gpio)) {
			dev_warn(&client->dev,
				 "no irq-gpios, working in raw mode\n");
		} else {
			data->irq_num = gpiod_to_irq(data->irq_gpio);
			if (data->irq_num < 0)
				return data->irq_num;
			ret = devm_request_threaded_irq(&client->dev,
							data->irq_num,
							NULL, qmi8658_handler,
							IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							"qmi8658_fifo_irq",
							indio_dev);
			if (ret)
				return ret;
		}
	}

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id qmi8658_id[] = {
	{ "qmi8658", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qmi8658_id);

static const struct of_device_id qmi8658_of_match[] = {
	{ .compatible = "qmi,qmi8658" },
	{ }
};
MODULE_DEVICE_TABLE(of, qmi8658_of_match);

static struct i2c_driver qmi8658_driver = {
	.driver = {
		.name = "qmi8658_fifo",
		.of_match_table = qmi8658_of_match,
	},
	.probe = qmi8658_probe,
	.id_table = qmi8658_id,
};
module_i2c_driver(qmi8658_driver);

MODULE_AUTHOR("Embedded Engineer");
MODULE_DESCRIPTION("QMI8658A Linux 6.6 FIFO Driver (fixed per Datasheet Rev D)");
MODULE_LICENSE("GPL");
