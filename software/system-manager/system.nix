{ lib, pkgs, ... }:
{
  config = {
    nixpkgs.hostPlatform = "aarch64-linux";

    # Enable and configure services
    services = {
      # nginx.enable = true;
    };

    environment = {
      # Packages that should be installed on a system
      systemPackages = [
        direnv
      ];

      # Add directories and files to `/etc` and set their permissions
      etc = {
        can-network-rule = {
          target = "./systemd/network/80-can.network";
          text = ''
            [Match]
            Name=can*

            [CAN]
            BitRate=500K
            RestartSec=1000ms
            BusErrorReporting=yes
            PresumeAck=yes
          '';
          mode = "0644";
        };
        can-udev-rule = {
          target = "/udev/rules.d/80-can-txqueuelen.rules";
          text = ''
            SUBSYSTEM=="net", ACTION=="add|change", KERNEL=="can*", ATTR{tx_queue_len}="128"
          '';
        };
      };
    };

    # Enable and configure systemd services
    systemd.services = { };

    # Configure systemd tmpfile settings
    systemd.tmpfiles = {
      # rules = [
      #   "D /var/tmp/system-manager 0755 root root -"
      # ];
      #
      # settings.sample = {
      #   "/var/tmp/sample".d = {
      #     mode = "0755";
      #   };
      # };
    };
  };
}
