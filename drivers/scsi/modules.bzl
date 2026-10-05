def register_modules(registry):
    registry.register(
        name = "drivers/scsi/sg",
        out = "sg.ko",
        config = "CONFIG_CHR_DEV_SG",
        srcs = [
            # do not sort
            "drivers/scsi/sg.c",
        ],
            deps = ["//vendor/lenovo/sm8850:private_module_headers"],
    )
