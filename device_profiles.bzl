# SPDX-License-Identifier: GPL-2.0-only
load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

def define_sm8850_profiles():
    string_flag(
        name = "sm8850_profile",
        build_setting_default = "baldur",
        visibility = ["//visibility:public"],
    )
    native.config_setting(
        name = "baldur_profile",
        flag_values = {":sm8850_profile": "baldur"},
        visibility = ["//visibility:public"],
    )
    native.config_setting(
        name = "wuji_profile",
        flag_values = {":sm8850_profile": "wuji"},
        visibility = ["//visibility:public"],
    )
