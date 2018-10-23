/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>

#include <asm/io.h>

#include "../codecs/tas5720.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_tas5720_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
};

static const struct snd_soc_dapm_widget imx_tas5720_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Output", NULL),
};

static int imx_audmux_config(int slave, int master)
{
    unsigned int ptcr, pdcr;
    slave = slave - 1;
    master = master - 1;
    
    ptcr = IMX_AUDMUX_V2_PTCR_SYN |
    IMX_AUDMUX_V2_PTCR_TFSDIR |
    IMX_AUDMUX_V2_PTCR_TFSEL(slave) |
    IMX_AUDMUX_V2_PTCR_TCLKDIR |
    IMX_AUDMUX_V2_PTCR_TCSEL(slave);
    pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(slave);
    imx_audmux_v2_configure_port(master, ptcr, pdcr);
    
    /*
     * According to RM, RCLKDIR and SYN should not be changed at same time.
     * So separate to two step for configuring this port.
     */
    ptcr |= IMX_AUDMUX_V2_PTCR_RFSDIR |
    IMX_AUDMUX_V2_PTCR_RFSEL(slave) |
    IMX_AUDMUX_V2_PTCR_RCLKDIR |
    IMX_AUDMUX_V2_PTCR_RCSEL(slave);
    imx_audmux_v2_configure_port(master, ptcr, pdcr);
    
    ptcr = IMX_AUDMUX_V2_PTCR_SYN;
    pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(master);
    imx_audmux_v2_configure_port(slave, ptcr, pdcr);
    
    return 0;
}

static int imx_tas5720_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params) {
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    u32 channels = params_channels(params);
    u32 rate = params_rate(params);
    u32 bclk = rate * channels * 32;
    int ret = 0;
    
    /* set cpu DAI configuration */
    ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
                              | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
    if (ret) {
        dev_err(cpu_dai->dev, "failed to set dai fmt\n");
        return ret;
    }
    
    ret = snd_soc_dai_set_tdm_slot(cpu_dai,
                                   channels == 1 ? 1 : 0x3,
                                   channels == 1 ? 1 : 0x3,
                                   2, 32);
    if (ret) {
        dev_err(cpu_dai->dev, "failed to set dai tdm slot\n");
        return ret;
    }
    
    ret = snd_soc_dai_set_sysclk(cpu_dai, 0, bclk, SND_SOC_CLOCK_OUT);
    if (ret)
        dev_err(cpu_dai->dev, "failed to set sysclk\n");
    
    return ret;
}

static int imx_tas5720_startup(struct snd_pcm_substream *substream) {
    volatile uint32_t* paddr;
    uint32_t dataWord;
    paddr = (volatile uint32_t*)ioremap(0x020C4060, 16);
    writel(0x011201F0, paddr);
    return 0;
}

static struct snd_soc_ops imx_tas5720_ops = {
    .hw_params = imx_tas5720_hw_params,
    .startup = imx_tas5720_startup,
};

static struct snd_soc_dai_link imx_dai = {
    .name = "imx-tas5720",
    .stream_name = "imx-tas5720",
    .codec_dai_name = "tas5720-amplifier",
    .dai_fmt    = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
    SND_SOC_DAIFMT_CBS_CFS,
    .ops = &imx_tas5720_ops,
};

static struct snd_soc_card snd_soc_card_imx_tas5720 = {
    .name = "imx-audio-tas5720",
    .dai_link = &imx_dai,
    .num_links = 1,
    .owner = THIS_MODULE,
};

static int imx_tas5720_probe(struct platform_device *pdev)
{
    struct snd_soc_card *card = &snd_soc_card_imx_tas5720;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct i2c_client *codec_dev;
	struct imx_tas5720_data *data = NULL;
	int int_port, ext_port;
	int ret;

    ret = of_property_read_u32(np, "mux-int-port", &int_port);
    if (ret) {
        dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
        return ret;
    }
    
    ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
    if (ret) {
        dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
        return ret;
    }
    
    imx_audmux_config(int_port, ext_port);

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

/*	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "tas5720";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_of_node = ssi_np;
	data->dai.platform_of_node = ssi_np;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			  SND_SOC_DAIFMT_CBS_CFS;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_tas5720_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_tas5720_dapm_widgets);
*/
    
    card->dev = &pdev->dev;
    card->dai_link->cpu_dai_name = dev_name(&ssi_pdev->dev);
    card->dai_link->platform_of_node = ssi_np;
    card->dai_link->codec_of_node = codec_np;
    
    platform_set_drvdata(pdev, card);
    
    ret = snd_soc_register_card(card);
    if (ret)
        dev_err(&pdev->dev, "Failed to register card: %d\n", ret);
    
	/*snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	of_node_put(ssi_np);
	of_node_put(codec_np);*/

	return 0;

fail:
    if (ssi_np)
        of_node_put(ssi_np);
    if (codec_np)
        of_node_put(codec_np);
	return ret;
}

static int imx_tas5720_remove(struct platform_device *pdev)
{
    struct snd_soc_card *card = &snd_soc_card_imx_tas5720;
    
    snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id imx_tas5720_dt_ids[] = {
	{ .compatible = "unu,imx-audio-tas5720", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tas5720_dt_ids);

static struct platform_driver imx_tas5720_driver = {
	.driver = {
		.name = "imx-tas5720",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_tas5720_dt_ids,
	},
	.probe = imx_tas5720_probe,
	.remove = imx_tas5720_remove,
};
module_platform_driver(imx_tas5720_driver);

MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Freescale i.MX tas5720 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-tas5720");
