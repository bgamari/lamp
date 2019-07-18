let nixpkgs = import <nixpkgs> {};
in with nixpkgs;
let
  gdb = nixpkgs.gdb.overrideAttrs (oldAttrs: {
    configureFlags = [ "--target=armv6-none-eabi" ];
  });

env = stdenv.mkDerivation {
  name = "firmware";
  nativeBuildInputs = [ gcc-arm-embedded gnumake openocd gdb ];
};
in env
