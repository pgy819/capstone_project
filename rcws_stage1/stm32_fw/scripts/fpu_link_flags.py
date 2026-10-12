"""PlatformIO extra script: apply the hard-float ABI to the link step as well."""
Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

FPU_FLAGS = ["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]

env.Append(LINKFLAGS=FPU_FLAGS)  # noqa: F821
