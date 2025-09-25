const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const mod = b.addModule("zimu", .{
        .root_source_file = null,
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });

    const lib = b.addLibrary(.{
        .name = "zimu",
        .root_module = mod,
        .linkage = .dynamic,
    });

    lib.linkLibC();
    lib.addCSourceFiles(.{
        .root = b.path("src"),
        .files = &.{
            "I2C.c",
            "ICM20948/DataConverter.c",
            "ICM20948/Icm20948Augmented.c",
            "ICM20948/Icm20948AuxCompassAkm.c",
            "ICM20948/Icm20948AuxTransport.c",
            "ICM20948/Icm20948DataBaseControl.c",
            "ICM20948/Icm20948DataBaseDriver.c",
            "ICM20948/Icm20948DataConverter.c",
            "ICM20948/Icm20948Dmp3Driver.c",
            "ICM20948/Icm20948LoadFirmware.c",
            "ICM20948/Icm20948MPUFifoControl.c",
            "ICM20948/Icm20948SelfTest.c",
            "ICM20948/Icm20948Setup.c",
            "ICM20948/Icm20948Transport.c",
            "ICM20948/Message.c",
        },
        .flags = &.{
            "-Wall",
            "-O2",
            "-fPIC",
            "-I./src",
            "-I./src/ICM20948",
        },
        .language = .c,
    });
    lib.addCSourceFiles(.{
        .root = b.path("src"),
        .files = &.{
            "IMU_ICM20948.cpp",
            "imu_c_api.cpp",
        },
        .flags = &.{
            "-Wall",
            "-O2",
            "-fPIC",
            "-std=c++17",
            "-fvisibility=hidden",
            "-I/src/include/linux",
            "-I./src",
            "-I./src/ICM20948",
            "-I/usr/include/arm-linux-gnueabihf/",
            "-I/usr/include/arm-linux-gnueabihf/c++/10/",
            "-I/usr/include/c++/10/",
        },
        .language = .cpp,
    });
    lib.installHeader(b.path("src/imu_c_api.h"), "imu_c_api.h");
    b.installArtifact(lib);
}
