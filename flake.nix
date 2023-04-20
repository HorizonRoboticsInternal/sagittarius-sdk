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
      # Explicitly use the develop branch of websocket++, because we need a fix
      # there that makes it compile under c++20.
      websocketpp = prev.websocketpp.overrideAttrs (old: {
        src = prev.fetchFromGitHub {
          owner = "zaphoyd";
          repo = "websocketpp";
          rev = "b9aeec6eaf3d5610503439b4fae3581d9aff08e8";  # develop branch
          hash = "sha256-qw5AbkFdL7hQuEp0ic/7r2BvqDMGEJvAeb1CdmOpnPw=";
        };
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

        mkDevShellOf = { python ? pkgs-dev.python3 }: pkgs-dev.mkShell.override {
          stdenv = pkgs-dev.overrideCC pkgs-dev.stdenv pkgs-dev.gcc12;
        } rec {
          name = "sagittarius_sdk";
          packages = with pkgs-dev; [
            # C/C++ Build Tools
            cmake
            cmakeCurses
            pkgconfig

            # C++ Library
            spdlog
            eigen
            boost
            websocketpp
            nlohmann_json
            libxcrypt

            # Python Environment
            (python.withPackages (pyPkgs: with pyPkgs; [
              pybind11
              numpy
              # Dev
              twine
              build
            ]))
          ];

          shellHook = ''
            export PS1="$(echo -e '\uf1c0') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
          '';
        };

    in {
      devShells.default = mkDevShellOf {};
      devShells.py38 = mkDevShellOf { python = pkgs-dev.python38; };
      devShells.py39 = mkDevShellOf { python = pkgs-dev.python39; };
    });
}
