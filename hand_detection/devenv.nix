{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:

{
  # Native shared libraries needed by the prebuilt wheels (numpy, opencv,
  # mediapipe). devenv adds these to LD_LIBRARY_PATH so the wheels can load
  # them at runtime.
  packages = [
    pkgs.zlib # libz.so.1 (numpy)
    pkgs.stdenv.cc.cc.lib # libstdc++.so.6
    pkgs.glib # libgthread/libglib (opencv)
    pkgs.libGL # libGL.so.1 (cv2.imshow)
    pkgs.glibc # general C runtime
  ];

  languages.python = {
    enable = true;
    version = "3.12";
    # Provide the manylinux compatibility libraries that prebuilt wheels
    # expect on Linux.
    manylinux.enable = true;
    venv = {
      enable = true;
      requirements = ./requirements.txt;
    };
  };
}
