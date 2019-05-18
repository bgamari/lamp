let 
  nixpkgs = 
    let src = fetchGit {
      url = "https://github.com/nixos/nixpkgs";
      rev = "de07276108906b189a7098d982e64e2ae720aca5";
      ref = "release-19.03";
    };
    in import src { overlays = [ (import rustOverlay) ]; };

  rustOverlay = fetchGit {
    url = "https://github.com/mozilla/nixpkgs-mozilla";
    rev = "9f35c4b09fd44a77227e79ff0c1b4b6a69dff533";
  };
in with nixpkgs; let
  rustChannel = rustChannelOf { date = "2019-05-11"; channel = "nightly"; };

in stdenv.mkDerivation {
  name = "rust";
  buildInputs = 
    let
    in [ rustup rustChannel.rust gcc-arm-embedded ];
  TARGET_CC = "${nixpkgs.gcc-arm-embedded}/bin/arm-none-eabi-gcc";
  TARGET = "thumbv7m-none-eabi";
}

# Build instructions:
#
# rustup target add $TARGET
# rustup update
# cargo install xargo
# xargo build
#
