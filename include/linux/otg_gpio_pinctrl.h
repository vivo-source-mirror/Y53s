/*
 * Copyright (C) 2016 vivo Inc.
 * This head list is used for otg gpio pull 
 * pin ctrl
 * modify by wutianwen
 */
 struct mtk_otg_gpio_pinctrl{
	 struct pinctrl *pinctrl;
	 unsigned int usbid_gpio;
	 struct pinctrl_state *otg_iddig_gpio;
	 struct pinctrl_state *otg_iddig_gpio_pull_down;
	 struct pinctrl_state *otg_pull_high;
	 struct pinctrl_state *otg_pull_low;
	 struct pinctrl_state *otg_pull_in;
	 struct pinctrl_state *otg_pull1_high;
	 struct pinctrl_state *otg_pull1_low;
	 struct pinctrl_state *otg_pull1_in;
	 struct pinctrl_state *otg_iddig_init;
	 struct pinctrl_state *otg_pull_down;
	 struct pinctrl_state *otg_pull1_down;
	 struct pinctrl_state *otg_iddig_pull_down;
};
