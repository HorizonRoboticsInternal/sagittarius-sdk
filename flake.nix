{
  description = "Sagittarius Arm SDK";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/22.11";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils, ... }: {
    # This is the overlay for our development environment.
    overlays.dev = final: prev: {
      lcm = prev.lcm.overrideAttrs (old: {
        outputs = [ "out" ];
      });
    };

    # This is the overlay that includes the final products.
    overlays.default = nixpkgs.lib.composeManyExtensions [
      self.overlays.dev
      (final: prev: {
        pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
          # TODO(breakds): Add the final python package here.
        ];
      })
    ];
  } // utils.lib.eachSystem [
    "x86_64-linux" "aarch64-linux"
  ] (system:
    let pkgs-dev = import nixpkgs {
          inherit system;
          overlays = [
            self.overlays.dev
          ];
        };

        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
          overlays = [
            self.overlays.default
          ];
        };
    in {
      devShells.default = pkgs-dev.mkShell rec {
        name = "sagittarius_sdk";
        packages = with pkgs-dev; [
          # C/C++ Build Tools
          llvmPackages_14.clang
          cmake
          cmakeCurses
          pkgconfig

          # C++ Library
          spdlog
          eigen
          boost
        ];

        shellHook = ''
          export CC=clang
          export CXX=clang++
          # Manually set where to look for libstdc++.so.6
          export LD_LIBRARY_PATH=${pkgs-dev.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH
          export PS1="$(echo -e '\uf1c0') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
         '';
      };
    });
}
