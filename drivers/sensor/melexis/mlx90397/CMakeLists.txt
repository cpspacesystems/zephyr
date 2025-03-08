# Copyright (c) Florian Weber <Florian.weber@live.de>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(mlx90397.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API mlx90397_async.c mlx90397_decoder.c)
