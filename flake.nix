{
  inputs = {
    nixpkgs.url = "nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    nix-arduino.url = "git+file:///home/qyriad/code/adafruit/nix-arduino";
  };

  outputs = { self, nixpkgs, flake-utils, nix-arduino }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        #pkgs = (import nixpkgs { inherit system; }).pkgsCross.arm-embedded;
        pkgs = import nixpkgs { inherit system; };
        targetPkgs = pkgs.pkgsCross.arm-embedded;


        adafruit_boards_index = "https://adafruit.github.io/arduino-board-index/package_adafruit_index.json";
        pwd = builtins.getEnv "PWD";
        directories_base = builtins.trace pwd "${pwd}/.arduino15";
        # PWD will be expanded in shellHook.

        samdPkgs = nix-arduino.packages.${system}.default;

        arduinoEnv = derivation {
          name = "arduino-env";
          inherit system;

          builder = pkgs.writeShellScript "arduino-env-builder.sh" ''
            ${pkgs.coreutils}/bin/mkdir -p "$out"
            ${pkgs.xorg.lndir}/bin/lndir "${samdPkgs}" "$out"
          '';
        };
      in
      {
        packages.default = pkgs.symlinkJoin {
          name = "arduino-sym-env";
          paths = [
            targetPkgs.pkgsBuildTarget.gcc
            pkgs.arduino
            pkgs.arduino-cli
            pkgs.clang-tools
            pkgs.just
          ];
        };
        devShells.default = pkgs.pkgsHostHost.mkShell {
          packages = [
            targetPkgs.pkgsBuildTarget.gcc
            #arduinoEnv
            pkgs.arduino
            pkgs.arduino-cli
            pkgs.clang-tools
            pkgs.just
            pkgs.doxygen
            pkgs.watchexec
          ];

          # Environment variables for configuration.

          #shellHook = ''
          #  set -xeuo pipefail
          #  [[ -a .arduino15 ]] && unlink .arduino15/packages || mkdir -p .arduino15
          #  ln -s "${arduinoEnv}" ".arduino15/packages"
          #  export ARDUINO_DIRECTORIES_DATA="$PWD/.arduino15"
          #  export ARDUINO_DIRECTORIES_DOWNLOADS="$PWD/.arduino15/staging"
          #  export ARDUINO_DIRECTORIES_USER="$PWD/Arduino"
          #  set +x
          #  unset shellHook
          #'';

          shellHook = ''
            set -x
            #rm -rf .arduino15
            export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS="${adafruit_boards_index}"
            export ARDUINO_DIRECTORIES_DATA="$PWD/.arduino15"
            export ARDUINO_DIRECTORIES_DOWNLOADS="$PWD/.arduino15/staging"
            export ARDUINO_DIRECTORIES_USER="$PWD/Arduino"

            arduino-cli core update-index

            arduino-cli --additional-urls ${adafruit_boards_index} core install arduino:avr adafruit:samd arduino:mbed_nano
            arduino-cli lib install "Adafruit BusIO"

            set +x
            unset shellHook
          '';
        };

      }
    )
  ;
}
