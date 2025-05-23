# Dependencies

1. [Install CTRE Phoenix 6](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-nonfrc.html) library:
    - Run (replace YEAR with year):
        ```
        YEAR=<year>
        sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
        sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"
        ```
        If you are installing on an arm machine, see the official documentation for changing the target distribution.
    - Run:
        ```
        sudo apt update
        sudo apt install phoenix6
        ```
