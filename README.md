 1.将驱动文件放到linux目录iio/imu下config选中
 2.dts配置如下
 &i2c2 {
	 status = "okay";
   qmi8658: qmi8658@6a {
       	compatible = "qmi,qmi8658";
       	reg = <0x6a>;
		    irq-gpios = <&gpio 74 GPIO_ACTIVE_HIGH>;
    	  irq-flags = <2>;
       	status = "okay";
   };

};
 
 
