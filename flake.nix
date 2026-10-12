{
  description = "ROS 2 jazzy + Mujoco Devshell";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    rospkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    nixpkgs.url = "github:nixos/nixpkgs/nixos-26.05";
    nixgl.url = "github:nix-community/nixGL";

    robot-control-msgs-src = {
      url = "github:dfki-ric-underactuated-lab/robot_control_msgs";
      flake = false;
    };
    franka-description-src = {
      url = "github:frankarobotics/franka_description/jazzy";
      flake = false;
    };

    unitree-ros2-src = {
      url = "github:unitreerobotics/unitree_ros2/66ae098";
      flake = false;
    };
  };

  outputs = { self, nixgl, nix-ros-overlay, rospkgs, nixpkgs, franka-description-src, robot-control-msgs-src, unitree-ros2-src }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import rospkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros2 = pkgs.rosPackages.jazzy;
        python = pkgs.python3;
        pythonPackages = pkgs.python3Packages;

        urdf-parser-py = pythonPackages.buildPythonPackage rec {
          pname = "urdf_parser_py";
          version = "0.0.4";
          src = pkgs.fetchPypi {
            inherit pname version;
            hash = "sha256-6YP2NxRf3tZ7z/alQjAgabuXWy7fGxgxjAk6u6G3lMw="; # Replace with real hash
          };
          nativeBuildInputs = with pythonPackages; [
            setuptools
            wheel
          ];
          buildInputs = with pythonPackages; [
            pyyaml
            lxml
          ];
          build-system = with pythonPackages; [
            setuptools
            wheel
          ];
          pyproject = true;
          doCheck = true;
        };

        coacd = pythonPackages.buildPythonPackage rec {
          pname = "coacd";
          version = "1.0.14";

          format = "wheel";

           src = pkgs.fetchurl {
            url = "https://files.pythonhosted.org/packages/88/7c/ec7edfc77c48fe97434ae813eca7614103289105bb546952ed9a0a266ce8/coacd-1.0.14-cp39-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl";
            hash = "sha256-bBNbcXrwllI7vr8f2NMm6JUMAkI3/pgGrUbe3YtMMfA=";
          };

          nativeBuildInputs = [
            pkgs.autoPatchelfHook
          ];
          buildInputs = [
            pkgs.zlib
          ];
          dependencies = [
            pythonPackages.numpy
          ];

          pythonImportsCheck = [ "coacd" ];
          doCheck = false;

          meta = {
            description =
              "Approximate convex decomposition with collision-aware concavity";
            homepage = "https://github.com/SarahWeiii/CoACD";
            license = pkgs.lib.licenses.mit;
            platforms = [ "x86_64-linux" ];
          };
        };

        unitree-hg-msgs = pkgs.stdenv.mkDerivation {
          pname = "unitree-hg-msgs";
          version = "0.3.0";
          src = "${unitree-ros2-src}/cyclonedds_ws/src/unitree/unitree_hg";

          nativeBuildInputs = [
            pkgs.cmake
            ros2.ament-cmake
            ros2.rosidl-default-generators
            ros2.rosidl-generator-dds-idl
            ros2.geometry-msgs
          ];

          buildInputs = [
            ros2.ament-cmake
          ];

          cmakeFlags = [
            "-DBUILD_TESTING=OFF"
          ];

        };

        robot-control-msgs = pkgs.stdenv.mkDerivation {
          pname = "robot-control-msgs";
          version = "0.0.0";
          src = robot-control-msgs-src;

          nativeBuildInputs = [
            pkgs.cmake
            ros2.ament-cmake
            ros2.rosidl-default-generators
          ];

          buildInputs = [
            ros2.ament-cmake
          ];

          propagatedBuildInputs = with ros2; [
            xacro
            rviz2
            joint-state-publisher-gui
            pkgs.qt5.wrapQtAppsHook
          ];

          cmakeFlags = [
            "-DBUILD_TESTING=OFF"
          ];
        };

        franka-description = pkgs.stdenv.mkDerivation {
          pname = "franka-description";
          version = "2.9.0";
          src = franka-description-src;

          nativeBuildInputs = [
            pkgs.cmake
            ros2.ament-cmake
            pkgs.qt5.wrapQtAppsHook
          ];

          buildInputs = [
            ros2.ament-cmake
          ];

          propagatedBuildInputs = with ros2; [
            xacro
            rviz2
            joint-state-publisher-gui
          ];

          cmakeFlags = [
            "-DBUILD_TESTING=OFF"
          ];

          meta = {
            description = "Official Franka robot descriptions for ROS 2";
            license = pkgs.lib.licenses.asl20;
          };
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ros2-jazzy-devshell";

          packages = [
            pkgs.gcc14
            pkgs.clang-tools # clangd for C++ LSP
            pkgs.cmake
            pkgs.gdb

            pkgs.glfw
            pkgs.mesa
            pkgs.libGL
            pkgs.vulkan-loader

            pkgs.qhull
            pkgs.tinyxml
            pkgs.tinyobjloader
            pkgs.libx11
            pkgs.libxinerama
            pkgs.libxcursor
            pkgs.nanoflann

            pkgs.pcl
            pkgs.octomap
            # kindr
            pkgs.boost
            pkgs.tbb

            pkgs.tcpdump

            (python.withPackages (ps: with ps; [
              scipy
              matplotlib
              transforms3d
              urdf-parser-py
              trimesh
              coacd
            ]))

            (ros2.buildEnv {
              underlay = true;
              paths = with ros2; [
                ros-core
                ament-cmake-core
                geometry-msgs
                tf2-ros
                python-cmake-module
                tf-transformations
                urdf
                robot-localization
                hardware-interface
                robot-state-publisher
                joint-state-broadcaster
                controller-manager
                realtime-tools
                rviz2
                xacro
                ros2-control
                ros2-controllers
                pcl-ros
                perception-pcl
                pcl-conversions
                cv-bridge

                # RQT
                rqt-robot-steering
                rqt-tf-tree
                rqt-image-view
                rqt-action
                rqt-bag
                rqt-publisher
                rqt-service-caller
                rqt-robot-monitor

                franka-description
                realsense2-description
                ur-description

                unitree-hg-msgs
                robot-control-msgs
              ];
            })

            pkgs.colcon
          ];

          shellHook = ''
            eval "$(register-python-argcomplete ros2)"
            eval "$(register-python-argcomplete colcon)"
            eval "$(register-python-argcomplete rosidl)"
            export QT_FONT_DPI=192
            export QT_QPA_PLATFORM=xcb
            export LD_LIBRARY_PATH=${pkgs.wayland}/lib:/run/opengl-driver/lib:$LD_LIBRARY_PATH
          '';
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
