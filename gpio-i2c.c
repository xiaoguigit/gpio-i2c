/*
 * gpio-i2c.c -- am335x platform gpio-i2c driver 
 * date:2017/12/19 
 * version:  v1.0.1
 * Copyright (C) 2017 Zuogui Yang <zuogui.yang@gzseeing.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/i2c-dev.h>

#define GPIO_I2C_CNT  5

#define GPIO_I2C_CMD_MAGIC 		'x'
#define GPIO_I2C_READ			_IO(GPIO_I2C_CMD_MAGIC, 0x01)
#define GPIO_I2C_WRITE			_IO(GPIO_I2C_CMD_MAGIC, 0x02)
#define GPIO_I2C_RDWR                   _IO(GPIO_I2C_CMD_MAGIC, 0x03)
#define I2C_RDWR	0x0707	/* Combined R/W transfer (one STOP only) */

#define ACK 0
#define NACK 1

static int major;      //am335x-gpio-i2c主设备号


//#define __DEBUG_AM335X__

#ifdef __DEBUG_AM335X__ 
#define debug(format,...) printk("LINE[%d]"format,  __LINE__, ##__VA_ARGS__)
#else 
#define debug(format,...)   
#endif 


struct am335x_gpio_i2c_platform_data {
    unsigned int    sda_pin;
    unsigned int	    scl_pin;
    int     udelay;
    int     timeout;
    int     dev_name_id;
    unsigned int	    sda_is_open_drain:1;
    unsigned int    scl_is_open_drain:1;
    unsigned int    scl_is_output_only:1;
    char name[128];
};


struct gpio_i2c_dev{
    struct am335x_gpio_i2c_platform_data data;
    struct cdev gpio_i2c_cdev;
    struct class *cls;
};


typedef struct{
    unsigned char addr;
    char r_buf[128];
    char w_buf[128];
    int r_len;
    int w_len;
    int offset;
}gpio_i2c_msg;


struct i2c_msg
{
    unsigned short addr;
    unsigned short flags;
    #define I2C_M_TEN 0x0010
    #define I2C_M_RD 0x0001
    unsigned short len;
    unsigned char *buf;
};

static void gpio_i2c_delay(struct gpio_i2c_dev* dev)
{
    udelay(dev->data.udelay);
}


//产生IIC起始信号
static void IIC_Start(struct gpio_i2c_dev* dev)
{
    gpio_i2c_delay(dev);
    gpio_direction_output(dev->data.sda_pin, 1);          //设置SDA方向为输出  
    gpio_direction_output (dev->data.scl_pin, 1);         //设置SCL方向为输出  
    gpio_set_value(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.sda_pin, 0);
    gpio_i2c_delay(dev);
}	

//产生IIC停止信号
static void IIC_Stop(struct gpio_i2c_dev* dev)
{
    gpio_direction_output(dev->data.sda_pin, 1);          //设置SDA方向为输出  
    gpio_set_value(dev->data.sda_pin, 0);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.sda_pin, 1);//发送I2C总线结束信号
    gpio_i2c_delay(dev);  						   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static unsigned char IIC_Wait_Ack(struct gpio_i2c_dev* dev)
{
    unsigned char ucErrTime=0;
    gpio_direction_input(dev->data.sda_pin);          //设置SDA方向为输入  
    gpio_i2c_delay(dev);	   
    gpio_set_value(dev->data.scl_pin, 1); 
    while(gpio_get_value(dev->data.sda_pin) )
    {
        ucErrTime++;
        if(ucErrTime>50)
        {
        	IIC_Stop(dev);
        	return 1;
        }
    }
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 0); 
//    ndelay(100);
    gpio_i2c_delay(dev);
    gpio_direction_output(dev->data.sda_pin, 1); 
    gpio_i2c_delay(dev);
    return 0;  
} 
//产生ACK应答
static void IIC_Ack(struct gpio_i2c_dev* dev)
{
    gpio_set_value(dev->data.scl_pin, 0);
    gpio_direction_output(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.sda_pin, 0);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 0);
}
//不产生ACK应答		    
static void IIC_NAck(struct gpio_i2c_dev* dev)
{
    gpio_set_value(dev->data.scl_pin, 0);
    gpio_direction_output(dev->data.sda_pin, 1);
    gpio_set_value(dev->data.sda_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 1);
    gpio_i2c_delay(dev);
    gpio_set_value(dev->data.scl_pin, 0);
}					 				     


static void IIC_Send_Byte(struct gpio_i2c_dev* dev, unsigned char txd)
{                        
    unsigned char t;       
    gpio_set_value(dev->data.scl_pin, 0);//拉低时钟开始数据传输
    gpio_i2c_delay(dev);
    gpio_direction_output(dev->data.sda_pin, 1); 	
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7)
            gpio_set_value(dev->data.sda_pin, 1);
        else
            gpio_set_value(dev->data.sda_pin, 0);
        txd<<=1; 	  
        gpio_i2c_delay(dev);  
        gpio_set_value(dev->data.scl_pin, 1);
        gpio_i2c_delay(dev); 
        gpio_set_value(dev->data.scl_pin, 0);	
        if(t == 7){
            ndelay(500); 
            gpio_direction_input(dev->data.sda_pin);
        }else{
            gpio_i2c_delay(dev);
        }
    }	 
} 	    

static unsigned char IIC_Read_Byte(struct gpio_i2c_dev* dev, unsigned char ack)
{
    unsigned char i,receive=0;
    gpio_direction_input(dev->data.sda_pin); //SDA设置为输入
    for(i=0;i<8;i++ )
    {
        gpio_set_value(dev->data.scl_pin, 0); 
        gpio_i2c_delay(dev);
        gpio_set_value(dev->data.scl_pin, 1);
        receive<<=1;
        if(gpio_get_value(dev->data.sda_pin) )
            receive++;   
        gpio_i2c_delay(dev); 
    }					 
    if (!ack)
        IIC_NAck(dev);//发送nACK
    else
        IIC_Ack(dev); //发送ACK   
    return receive;
}


static  int i2c_read_register(struct gpio_i2c_dev* dev, unsigned char device_addr, unsigned char offset, unsigned char *buf, int len)
 {
    int ack = 1;
    int i = 0;
    
    IIC_Start(dev);  
    IIC_Send_Byte(dev, (device_addr & 0xfe));   //发送器件地址
    ack = IIC_Wait_Ack(dev); 
    if(ack){
        return -1;
    }
    IIC_Send_Byte(dev,offset);
    ack = IIC_Wait_Ack(dev);
    if(ack){
        return -1;
    }
    IIC_Start(dev);
    IIC_Send_Byte(dev, (device_addr & 0xfe) | 0x01);   //发送器件地址
    ack = IIC_Wait_Ack(dev);
    if(ack){
        return -1;
    }
    for(i = 0; i < len -1; i++){
        buf[i]=IIC_Read_Byte(dev, 1);
    } 
    buf[i]=IIC_Read_Byte(dev, 0);
    IIC_Stop(dev);//产生一个停止条件	    
    return 0;
 }



static  int i2c_write_register(struct gpio_i2c_dev* dev, unsigned char device_addr, unsigned char offset, unsigned char *buf, int len)
{
    int i = 0;
    int ack = 1;

    debug("device_addr = %02x\n", device_addr);
    debug("offset = %02x\n", offset);
    debug("data : ");
    for(i = 0; i < len; i++){
        debug("%02x ", buf[i]);
    }
    debug("\n");

    IIC_Start(dev);  
    IIC_Send_Byte(dev, device_addr);   //发送器件地址0XA0,写数据 
    ack = IIC_Wait_Ack(dev);
    if(ack){
        debug("%d addr no ack\n", __LINE__);
        return -1;
    }
    IIC_Send_Byte(dev, offset);   //发送低地址
    ack = IIC_Wait_Ack(dev);
    if(ack){
        debug("%d offset no ack\n", __LINE__);
        return -1;
    } 
    for(i = 0; i < len; i++){
        IIC_Send_Byte(dev, buf[i]);     //发送字节							   
        ack = IIC_Wait_Ack(dev);
        if(ack){
            debug("%d data %d no ack\n", __LINE__, i);
            return -1;
        }
    }
    IIC_Stop(dev);//产生一个停止条件 
    return 0;
}



static  int i2c_read_write(struct gpio_i2c_dev* dev, unsigned int slave_addr, unsigned char *write_data, int write_len, unsigned char *read_data, int read_len)
 {
    int ack = 1;
    int i = 0;

    debug("device_addr = %02x\n", slave_addr);
    debug("offset = %02x\n", write_data[0]);
    debug("data : ");
    for(i = 0; i < write_len - 1; i++){
        debug("%02x ", write_data[i+1]);
    }
    debug("\n");

    IIC_Start(dev);  
    IIC_Send_Byte(dev, (slave_addr & 0xfe));   //发送器件地址
    ack = IIC_Wait_Ack(dev); 
    if(ack){
        debug("%d addr no ack\n", __LINE__);
        return -1;
    }
    for(i = 0; i < write_len; i++){
        IIC_Send_Byte(dev, write_data[i]);
        ack = IIC_Wait_Ack(dev);
        if(ack){
            debug("%d data %d no ack\n", __LINE__, i);
            return -1;
        }
    }

    IIC_Start(dev);
    IIC_Send_Byte(dev, (slave_addr & 0xfe) | 0x01);   //发送器件地址
    ack = IIC_Wait_Ack(dev);
    if(ack){
        debug(" %d restart addr no ack\n", __LINE__);
        return -1;
    }
    debug("restart read %02x\n", (slave_addr & 0xfe) | 0x01);
    debug("read len = %d\n", read_len -1);

    for(i = 0; i < read_len -1; i++){
        read_data[i]=IIC_Read_Byte(dev, 1);
    } 
    read_data[i]=IIC_Read_Byte(dev, 0);
    IIC_Stop(dev);//产生一个停止条件	    
    return 0;
 }



#if 0

static void gpio_i2c_mdelay(int ms)
{
    mdelay(ms);
}


 unsigned char AT24CXX_ReadOneByte(struct gpio_i2c_dev* dev, unsigned int ReadAddr)
 {				  
    unsigned char temp=0;		  	    																 
    IIC_Start(dev);  
    IIC_Send_Byte(dev, 0Xa0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	 

    IIC_Wait_Ack(dev); 
    IIC_Send_Byte(dev, ReadAddr%256);   //发送低地址
    IIC_Wait_Ack(dev);	    
    IIC_Start(dev);  	 	   
    IIC_Send_Byte(dev, 0Xa1);           //进入接收模式			   
    IIC_Wait_Ack(dev);	 
    temp=IIC_Read_Byte(dev, 0);		   
    IIC_Stop(dev);//产生一个停止条件	    
    return temp;
}

void AT24CXX_WriteOneByte(struct gpio_i2c_dev* dev, unsigned int WriteAddr,unsigned char DataToWrite)
{				   	  	    																 
    IIC_Start(dev);  
    IIC_Send_Byte(dev, 0Xa0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 
    IIC_Wait_Ack(dev);	   
    IIC_Send_Byte(dev, WriteAddr%256);   //发送低地址
    IIC_Wait_Ack(dev); 	 										  		   
    IIC_Send_Byte(dev, DataToWrite);     //发送字节							   
    IIC_Wait_Ack(dev);  		    	   
    IIC_Stop(dev);//产生一个停止条件 
    gpio_i2c_mdelay(10);	 
}

#endif


static int gpio_i2c_open(struct inode *inode, struct file *file)
{
    struct gpio_i2c_dev *dev = container_of(inode->i_cdev, struct gpio_i2c_dev, gpio_i2c_cdev);
    debug("gpio_i2c_open\n");
    file->private_data = dev;
    debug ("open %s\n", dev->data.name);
    return 0;
}



static long gpio_i2c_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    unsigned char new_addr;
    gpio_i2c_msg msg;
    struct i2c_rdwr_ioctl_data rdwr_arg;

    struct gpio_i2c_dev *dev = filp->private_data;

    if(_IOC_TYPE(cmd) != GPIO_I2C_CMD_MAGIC && cmd != I2C_RDWR) 
        return - EINVAL;
    
    memset(&msg, 0, sizeof(msg));
    memset(&rdwr_arg, 0, sizeof(rdwr_arg));

    
    switch(cmd)
    {
        case GPIO_I2C_READ:
            //printk("at24cxx : %02x\n", AT24CXX_ReadOneByte(dev, 0x02));
            ret = copy_from_user(&msg, (struct gpio_i2c_msg*)arg, sizeof(msg));
            if(ret < 0){
                return ret;
            }
            if(i2c_read_register(dev, msg.addr, msg.offset, msg.r_buf, msg.r_len)){
                return -1;
            }
            ret = copy_to_user((struct gpio_i2c_msg*)arg, &msg, sizeof(msg));
            if(ret < 0){
                return ret;
            }
            break;
        case GPIO_I2C_WRITE:
            //AT24CXX_WriteOneByte(dev, 0x02, 0xa3);
            ret = copy_from_user(&msg, (struct gpio_i2c_msg*)arg, sizeof(msg));
            if(ret < 0){
                return ret;
            }
            if(i2c_write_register(dev, msg.addr, msg.offset, msg.w_buf, msg.w_len)){
                return -1;
            }
            break;
        case GPIO_I2C_RDWR:
            ret = copy_from_user(&msg, (struct gpio_i2c_msg*)arg, sizeof(msg));
            if(ret < 0){
                return ret;
            }
            if(i2c_read_write(dev, msg.addr, msg.w_buf, msg.w_len, msg.r_buf, msg.r_len)){
                return -1;
            }
            ret = copy_to_user((struct gpio_i2c_msg*)arg, &msg, sizeof(msg));
            if(ret < 0){
                return ret;
            }
            break;
         case I2C_RDWR:
            ret = copy_from_user(&rdwr_arg, (struct i2c_rdwr_ioctl_data*)arg, sizeof(rdwr_arg));
            if(ret < 0){
                return ret;
            }
            new_addr = (rdwr_arg.msgs[0].addr << 1) & 0xfe;
            if(rdwr_arg.nmsgs == 2){
                if(i2c_read_write(dev, new_addr, rdwr_arg.msgs[0].buf, rdwr_arg.msgs[0].len, 
                        rdwr_arg.msgs[1].buf, rdwr_arg.msgs[1].len)){
                    return -1;
                }
            }

            if(rdwr_arg.nmsgs == 1  && rdwr_arg.msgs[0].flags  == 1){
                if(i2c_read_register(dev, new_addr, rdwr_arg.msgs[0].buf[0], rdwr_arg.msgs[0].buf, rdwr_arg.msgs[0].len)){
                    return -1;
                }
            }

            if(rdwr_arg.nmsgs == 1  && rdwr_arg.msgs[0].flags  == 0){
                if(i2c_write_register(dev, new_addr, rdwr_arg.msgs[0].buf[0], &(rdwr_arg.msgs[0].buf[1]), rdwr_arg.msgs[0].len - 1)){
                    return -1;
                }
            }

            ret = copy_to_user((struct rdwr_arg*)arg, &rdwr_arg, sizeof(rdwr_arg));
            if(ret < 0){
                return ret;
            }
            break;
        default:
            break;
    }
    return 0;
}

static struct file_operations gpio_i2c_fops = {
	.owner = THIS_MODULE,
	.open  = gpio_i2c_open,
	.unlocked_ioctl = gpio_i2c_ioctl,

};



static void of_i2c_gpio_get_props(struct device_node *np,
				  struct am335x_gpio_i2c_platform_data *pdata)
{
	u32 reg;
        of_property_read_u32(np, "dev-name,id-num", &pdata->dev_name_id);

	of_property_read_u32(np, "i2c-gpio,delay-us", &pdata->udelay);

	if (!of_property_read_u32(np, "i2c-gpio,timeout-ms", &reg))
		pdata->timeout = msecs_to_jiffies(reg);

	pdata->sda_is_open_drain =
		of_property_read_bool(np, "i2c-gpio,sda-open-drain");
	pdata->scl_is_open_drain =
		of_property_read_bool(np, "i2c-gpio,scl-open-drain");
	pdata->scl_is_output_only =
		of_property_read_bool(np, "i2c-gpio,scl-output-only");
}


static int of_i2c_gpio_get_pins(struct device_node *np,
				unsigned int *sda_pin, unsigned int *scl_pin)
{
	if (of_gpio_count(np) < 2)
		return -ENODEV;

        if(sda_pin == NULL || scl_pin == NULL){
            return -ENODEV;
        }

	*sda_pin = of_get_gpio(np, 0);
	*scl_pin = of_get_gpio(np, 1);

	if (*sda_pin == -EPROBE_DEFER || *scl_pin == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (!gpio_is_valid(*sda_pin) || !gpio_is_valid(*scl_pin)) {
		pr_err("%s: invalid GPIO pins, sda=%d/scl=%d\n",
		       np->full_name, *sda_pin, *scl_pin);
		return -ENODEV;
	}

        debug("sda: %d\n", *sda_pin);
        debug("scl: %d\n", *scl_pin);
        
	return 0;
}


static int am335x_i2c_gpio_probe(struct platform_device *pdev)
{
    struct gpio_i2c_dev* priv;
    int ret;
    int i;
    dev_t devid;
    priv = devm_kzalloc(&pdev->dev, sizeof(struct gpio_i2c_dev), GFP_KERNEL);
    
    debug("##am335x_i2c_gpio_probe %s##\n", pdev->dev.of_node->name);
    if (pdev->dev.of_node) {
        ret = of_i2c_gpio_get_pins(pdev->dev.of_node, &(priv->data.sda_pin), &(priv->data.scl_pin));
        if (ret){
            debug("of_i2c_gpio_get_pins error\n");
            return ret;
        }
        of_i2c_gpio_get_props(pdev->dev.of_node, &priv->data);
    } 

    ret = devm_gpio_request(&pdev->dev, priv->data.sda_pin, "sda");
    if (ret) {
        if (ret == -EINVAL){
            ret = -EPROBE_DEFER;	/* Try again later */
        }
        debug("cannot request sda\n");
        return ret;
    }
    ret = devm_gpio_request(&pdev->dev, priv->data.scl_pin, "scl");
    if (ret) {
        if (ret == -EINVAL){
            ret = -EPROBE_DEFER;	/* Try again later */
        }
        debug("cannot request sda\n");
        return ret;
    }

    snprintf(priv->data.name, sizeof(priv->data.name), "i2c-gpio%d", priv->data.dev_name_id);
    debug("am335x %s\n", priv->data.name);

    if (major) {
    	devid = MKDEV(major, 0);
    	register_chrdev_region(devid, GPIO_I2C_CNT, priv->data.name);  
    } else {
    	alloc_chrdev_region(&devid, 0, GPIO_I2C_CNT, priv->data.name); 
    	major = MAJOR(devid);                     
    }

    cdev_init(&priv->gpio_i2c_cdev, &gpio_i2c_fops);
    cdev_add(&priv->gpio_i2c_cdev, MKDEV(major, priv->data.dev_name_id), GPIO_I2C_CNT);

    priv->cls = class_create(THIS_MODULE, priv->data.name);
    device_create(priv->cls, NULL, MKDEV(major, priv->data.dev_name_id), NULL, priv->data.name); /* /dev/gpio_i2c0 */


    platform_set_drvdata(pdev, priv);

    
    gpio_direction_output(priv->data.sda_pin, 1);          //设置SDA方向为输出  
    gpio_direction_output (priv->data.scl_pin, 1);         //设置SCL方向为输出  

    for(i =0; i < 10; i++){
        gpio_set_value(priv->data.scl_pin, 0);
        gpio_i2c_delay(priv);
        gpio_set_value(priv->data.scl_pin, 1);
        gpio_i2c_delay(priv);
    }
    
    return 0;
}

static int am335x_i2c_gpio_remove(struct platform_device *pdev)
{
    struct gpio_i2c_dev *priv;
    priv = platform_get_drvdata(pdev);
    debug("##am335x_i2c_gpio_remove##\n");
    devm_gpio_free(&pdev->dev, priv->data.sda_pin);
    devm_gpio_free(&pdev->dev, priv->data.scl_pin);

    device_destroy(priv->cls, MKDEV(major, priv->data.dev_name_id));
    class_destroy(priv->cls);

    cdev_del(&priv->gpio_i2c_cdev);
    unregister_chrdev_region(MKDEV(major, priv->data.dev_name_id), GPIO_I2C_CNT);
    devm_kfree(&pdev->dev, priv);
    return 0;
}


#if defined(CONFIG_OF)
static const struct of_device_id am335x_i2c_gpio_dt_ids[] = {
    { .compatible = "am335x-gpio-i2c", },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, am335x_i2c_gpio_dt_ids);
#endif



static struct platform_driver am335x_i2c_gpio_driver = {
    .driver		= {
    	.name	= "am335x-gpio-i2c",
    	.of_match_table	= of_match_ptr(am335x_i2c_gpio_dt_ids),
    },
    .probe		= am335x_i2c_gpio_probe,
    .remove		= am335x_i2c_gpio_remove,
};



static int gpio_i2c_init(void)
{
    int ret;

    ret = platform_driver_register(&am335x_i2c_gpio_driver);
    if (ret){
        printk("am335x_i2c-gpio: probe failed: %d\n", ret);
        return ret;
    }
    return 0;
}

static void gpio_i2c_exit(void)
{
    platform_driver_unregister(&am335x_i2c_gpio_driver);
}


module_init(gpio_i2c_init);
module_exit(gpio_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zuogui.Yang");
MODULE_DESCRIPTION("Am335x Platform gpio-i2c Driver");
MODULE_VERSION("1.0.1-2017-12-19");