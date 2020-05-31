/*
 * @file RoHM BD71827 LED driver
 *
 * Copyright (C) 2017
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/mfd/bd71827.h>

#define AMBER_BIT 7
#define GRN_BIT   6

struct bd71827_led_data {
	struct led_classdev cdev;
	char *name;
	int bit;
};

struct bd71827_led_data bd71827_leds[] = {
	{
		.name = "green",
		.bit = GRN_BIT,
	},
	{
		.name = "amber",
		.bit = AMBER_BIT,
	},
};

static struct bd71827 *bd71827_mfd = NULL;

#define NUMS_LED (sizeof(bd71827_leds)/sizeof(struct bd71827_led_data))

static void bd71827_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct bd71827_led_data *ldata = container_of(led_cdev,
						      struct bd71827_led_data, cdev);
	int on = (value > 0)? 1 : 0;
	bd71827_update_bits(bd71827_mfd, BD71827_REG_LED_CTRL,
			   (1 << ldata->bit), (on << ldata->bit));
}

static int bd71827_led_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < NUMS_LED; i++) {
		led_classdev_unregister(&bd71827_leds[i].cdev);
	}
	return 0;
}

static int bd71827_led_probe(struct platform_device *pdev)
{
	int err, i;

	bd71827_mfd = dev_get_drvdata(pdev->dev.parent);
	if (unlikely(!bd71827_mfd))
		return (-EINVAL);

	for (i = 0; i < NUMS_LED; i++) {
		bd71827_leds[i].cdev.name = bd71827_leds[i].name;
		bd71827_leds[i].cdev.brightness_set = bd71827_led_set;

		err = led_classdev_register(&pdev->dev, &bd71827_leds[i].cdev);
		if (err < 0)
			goto exit;
	}
	return 0;

exit:
	while (i--)
		led_classdev_unregister(&bd71827_leds[i].cdev);
	return err;
}

static struct platform_driver bd71827led_driver = {
	.probe		= bd71827_led_probe,
	.remove		= bd71827_led_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "bd71827-led",
	}
	,
};

static int __init bd71827_led_init(void)
{
	return platform_driver_register(&bd71827led_driver);
}

module_init(bd71827_led_init);

static void __exit bd71827_led_exit(void)
{
	platform_driver_unregister(&bd71827led_driver);
}
module_exit(bd71827_led_exit);

MODULE_AUTHOR("Mike Tsai <miketsai@amazon.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71827-led");
