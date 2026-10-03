{
  description = "Packages VESC firmware into a flake.";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-26.05";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };
        bldc-fw = pkgs.callPackage ./pkgs/bldc.nix {
          src = self;
        };
      in
      {
        packages = {
          inherit bldc-fw;
          default = bldc-fw;
        };

        # `nix develop`, and what CI runs tests/check.sh inside.
        #
        # Without this, nix develop fell back to the firmware package's build
        # environment: the ARM toolchain was there, but nothing else was, so
        # four of check.sh's stages skipped on every local run -- the gtest
        # host test, the QEMU suite, cppcheck and clang-tidy. A stage that
        # skips locally and runs in CI is a stage you find out about from a
        # red build.
        #
        # The analysis tools are pinned here deliberately, not incidentally.
        # They are version-sensitive: Ubuntu's cppcheck 2.13 flags two sites
        # in tests/app_pas/fixture.c that 2.21 does not, and its clang-tidy
        # reports a va_list in that same file as uninitialized where clang 21
        # does not. Pinning them is what makes "it passed locally" mean
        # something about CI.
        devShells.default = pkgs.mkShell {
          inputsFrom = [ bldc-fw ];

          packages = with pkgs; [
            # tests/qemu runs the real kernel on a simulated STM32F405. It
            # needs the ARM toolchain too, which comes from bldc-fw above.
            qemu

            # The analysis stages.
            cppcheck
            clang-tools

            # tests/utils_math is a gtest suite.
            gtest

            # tests/uavcan_vesc_frames builds -m32: libcanard needs a 32-bit
            # host, which is what upstream's Dockerfile for it exists to
            # provide. gcc_multi carries the 32-bit glibc headers, without
            # which the build stops at gnu/stubs-32.h.
            gcc_multi

            # proofs/: bounded model checking of the claims a few commit
            # messages make. See proofs/README.md.
            cbmc

            # gcc_multi rearranges library resolution enough that the
            # sanitizer runtimes stop finding libstdc++, which the three
            # script tests link through libubsan. Putting it back explicitly
            # is cheaper than giving up the 32-bit build.
            stdenv.cc.cc.lib

            python3
          ];
        };
      }
    )
    // {
      overlays.default = final: _prev: {
        bldc-fw = final.callPackage ./pkgs/bldc.nix {
          src = self;
        };
      };
    };
}
