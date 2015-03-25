#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
//*****************for debug *****************//
#define DEBUG_SWITCH_CAPSENSOR 1
//open the debug log
#if    DEBUG_SWITCH_CAPSENSOR
#define CAPSENSOR_DEBUG(fmt,arg...)      printk("<<-CAP SENSOR->> "fmt"\n",##arg)
#else
#define CAPSENSOR_DEBUG(fmt,args...) /*do nothing */
#endif

#define CAP_PRINT(fmt,args...) printk("<<-CAP SENSOR->>"fmt"\n",##arg)

//****************Variables and define****************//
/*
 capsensor_enable TRUE
 capsensor_disable FALSE
*/
bool capsensor_switch=FALSE;

/*
    capsensor_near TRUE
    capsensor_away FALSE
*/
bool capsensor_near=FALSE;

#define GPIO_CAP_EINT_PIN GPIO20  //wabgliangfeng 20140411 modify

//*****************sys file system start*****************//
static ssize_t show_CapSensor_Switch(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(TRUE==capsensor_switch)
	  return  sprintf(buf, "Capsensor is enabled.\n");
	else
	  return  sprintf(buf, "Capsensor is disabled.\n");
}
static ssize_t store_CapSensor_Switch(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	if((strcmp(buf,"1\n")==0)||(strcmp(buf,"enable\n")==0))
	  {
	  	CAPSENSOR_DEBUG("power on iqs128.\n");
        if(TRUE !=hwPowerOn(MT6323_POWER_LDO_VCAM_AF, 3300,"iqs128")){
        	printk("<<-CAPSENSOR->> Fail to open digital power VCAM_A2(iqs128)\n");
        	return -1;
        }    
		capsensor_switch=TRUE;
	  }
	else if((strcmp(buf,"0\n")==0)||(strcmp(buf,"disable\n")==0))
	  {   
	  	CAPSENSOR_DEBUG("power off iqs128.\n");
         if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAM_AF,"iqs128")) {
         	printk("<<-CAPSENSOR->> Fail to OFF analog power\n");
         	return -1;   
         }   
		 capsensor_switch=FALSE;
	  } 
	else
	  {
		printk("<<-CAPSENSOR->> your input capsensor_switch =%s data is error\n",buf);
	  }
		  
    return size;
}

/*
* CapSensor_Data:
*664:  debug capsensor use adb command
*644:  just for meeting CTS test
*/
static DEVICE_ATTR(CapSensor_Switch, 0644, show_CapSensor_Switch, store_CapSensor_Switch);


static ssize_t show_CapSensor_Data(struct device *dev,struct device_attribute *attr, char *buf)
{
	  
	if(TRUE==capsensor_switch)
	  {
		 
		if(mt_get_gpio_in(GPIO_CAP_EINT_PIN)==1)
		   {  
			   CAPSENSOR_DEBUG("away");
			   capsensor_near=FALSE;
			   return sprintf(buf,"removed\n");
			}
		  else
		   {   CAPSENSOR_DEBUG("near");
			  capsensor_near=TRUE;
			  return sprintf(buf,"near\n");
		   } 
	  }
	else
	{
		CAPSENSOR_DEBUG("Capsensor is disabled,please enable it!");
	  	return  sprintf(buf, "Capsensor is disabled,please enable it!\n");
    }
}

/*
* CapSensor_Data:
*664:  debug capsensor use adb command
*644:  just for meeting CTS test
*/
static DEVICE_ATTR(CapSensor_Data,  0644, show_CapSensor_Data, NULL);
//*****************sys file system end*****************//
static void hw_init()
{
 	mt_set_gpio_mode(GPIO_CAP_EINT_PIN,GPIO_CAP_EINT_PIN_M_GPIO);
 	mt_set_gpio_dir(GPIO_CAP_EINT_PIN,GPIO_DIR_IN);
	//power on for capsensor
	if(TRUE !=hwPowerOn(MT6323_POWER_LDO_VCAM_AF, 3300,"iqs128")){
       printk("<<-CAPSENSOR->> Fail to open digital power VCAM_A2(iqs128)\n");
       return -1;
    } 
	capsensor_switch = TRUE; //enbale the read data switch
}


static int iqs128_prob(struct platform_device *dev)
{ 
  int ret_device_file=0;
  CAPSENSOR_DEBUG("iqs128_prob");
  hw_init();
  ret_device_file = device_create_file(&(dev->dev), &dev_attr_CapSensor_Switch);
  ret_device_file = device_create_file(&(dev->dev), &dev_attr_CapSensor_Data);
  return 0;
}

static int iqs128_remove(struct platform_device *dev)    
{
    CAPSENSOR_DEBUG("iqs128_remove");
	
    return 0;
}

static void iqs128_shutdown(struct platform_device *dev)    
{
     CAPSENSOR_DEBUG("iqs128_shutdown");
     if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAM_AF,"iqs128")) {
       printk("<<-CAP SENSOR->> Fail to OFF analog power\n");
       return -1;   
     }   
     return 0;
}

static int iqs128_suspend(struct platform_device *dev, pm_message_t state)    
{
    CAPSENSOR_DEBUG("iqs128_suspend");
    return 0;
}

static int iqs128_resume(struct platform_device *dev)
{
   CAPSENSOR_DEBUG("iqs128_resume");
   return 0;
}

struct platform_device capsensor_device = {
    .name   = "capsensor",
    .id	    = -1,
};

static struct platform_driver capsensor_driver = {
    .probe       = iqs128_prob,
    .remove      = iqs128_remove,
    .shutdown    = iqs128_shutdown,
    .suspend     = iqs128_suspend,
    .resume      = iqs128_resume,
    .driver      = {
    .name = "capsensor",
    },
};

static void capsensor_iqs128_init(void)
{
    int ret;
    ret = platform_device_register(&capsensor_device);
    if (ret) {
    printk("<<-CAPSENSOR->> Unable to register device (%d)\n", ret);
	return ret;
    }
    
    ret = platform_driver_register(&capsensor_driver);
    if (ret) {
    printk("<<-CAPSENSOR->> Unable to register driver (%d)\n", ret);
	return ret;
    }
    return 0;    
}

static void capsensor_iqs128_exit(void)
{
	CAPSENSOR_DEBUG("capsensor_iqs128_exit");
	platform_device_unregister(&capsensor_device);
	platform_driver_unregister(&capsensor_driver);

}


fs_initcall(capsensor_iqs128_init);
module_exit(capsensor_iqs128_exit);

MODULE_AUTHOR("zhaozhenfei@huaqin.com");
MODULE_DESCRIPTION("capsensor iqs128 Device Driver");
MODULE_LICENSE("GPL");
