# Jetson camera kernel patches

## `nv_imx477-probe-cleanup.patch`

Target: Jetson Linux `5.15.185-tegra`, NVIDIA OOT source branch
`l4t/l4t-r36.5`.

The vendor IMX477 probe path registers a `tegracam_device` before reading the
sensor ID. If that I2C read fails, the unpatched error path returns without
unregistering the device. Its `cam_reset_gpio` remains requested even after
`nv_imx477` is unloaded, so every later probe fails with `reset_gpio (-16)`
until the machine is rebooted.

The patch mirrors the cleanup used by NVIDIA's IMX219 driver and adds a module
version that `scripts/start_all.sh` uses to enable safe background retries.

Installed module version:

```text
2.0.6-camhj1-cleanup1
```

The original binary on this machine is backed up at:

```text
/lib/modules/5.15.185-tegra/updates/drivers/media/i2c/nv_imx477.ko.before-camhj1-cleanup1-20260805
```

After replacing the module, run `depmod -a 5.15.185-tegra`. A GPIO already
leaked by the old module is kernel state and still requires one reboot; later
failed probes can be unloaded and retried without rebooting.
