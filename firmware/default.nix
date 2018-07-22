with (import <nixpkgs> {});

stdenv.mkDerivation {
  name = "firmware";
  nativeBuildInputs = [ gnumake gcc-arm-embedded openocd ];
}
