##Install WSL 2
<details>
<summary>Expand for details</summary>
<details>
<summary>#### Enable Virtualization on Windows (Enabled by default on Windows manufactured PCs)</summary>
		<details>
		<summary>Access the UEFI/BIOS</summary>
			Open settings app > System > Recovery
			Select Recovery Options > Advanced Startup > Restart Now
			Choose Troubleshoot > Advanced options > UEFI Firmware Settings > Restart
			UEFI utility will open post restart
    	</details>
		<details>
		<summary>Make changes in the UEFI/BIOS (Based on Manufacturer)</summary>
			<details>
			<summary>Acer (Intel processor)</summary>
				Select Advanced Page in BIOS
				Enable the Intel VTX and Intel VTD settings (On certain devices press Ctrl+S first)
				Press F10
            </details>
			<details>
			<summary>Acer (AMD processor)</summary>
				Select Advanced Page in BIOS
				Enable the AMD-SVM and AMD-IOMMU settings (On certain devices press Ctrl+S first)
				Press F10
            </details>
			<details>
			<summary>ASUS (AMD Notebook)</summary>
				Select Advanced Page in BIOS
				Enable SVM mode
				Press F10
            </details>
			<details>
			<summary>ASUS (Intel Notebook)</summary>
				Select Advanced Page in BIOS
				Enable Virtualization Technology mode
				Press F10
            </details>
			<details>
			<summary>ASUS (Intel Motherboard)</summary>
				Press F7 to enter Advanced Mode (ROG motherboards enter directly, no need to F7)
				Select Advanced Page > CPU Configuration
				Enable Intel VMX Virtualization Technology (Enabled by default)
            </details>
			<details>
			<summary>Dell</summary>
				Select Advanced page/Virtualization Support in BIOS > Virtualization
				Enable Virtualization
				Set Virtualization for Direct-IO or VT-d to Enabled
				Press F10
            </details>
			<details>
			<summary>HP (OMEN by HP, Victus by HP, HP Spectre, HP ENVY, and HP Pavilion series)</summary>
				Select Configuration in BIOS settings > Virtualization Technology
				Enable Virtualization Technology
				Press F10
            </details>
			<details>
			<summary>HP (HP EliteBook, HP EliteDesk, HP ZHAN Notebook series)</summary>
				Select Advanced page in BIOS > System Options > press Enter
				Enable Virtualization Technology (VTx)
				Press F10
            </details>
			<details>
			<summary>HP (HP workstations)</summary>
				Select Security tab in BIOS > USB Security > press Enter
				Enable Virtualization Technology (VTx)
				Press F10
            </details>
			<details>
			<summary>Lenovo (Thinkpad)</summary>
				Select Security page in BIOS
				Enable Intel(R) Virtualization Technology and Intel VT-d Feature
				Press F10
            </details>
			<details>
			<summary>Lenovo (Ideapad)</summary>
				Select Configuration page in BIOS
				Enable AMD V(TM) Technology
				Press F10
            </details>
			<details>
			<summary>Lenovo (ThinkCentre)</summary>
				Select Advanced page in BIOS
				Enable Intel (R) Virtualization Technology
				Press F10
            </details>
			<details>
			<summary>Unlisted Manufacturers</summary>
				Refer to device's firmware documentation
            </details>
        </details>
        <details>
		<summary>Enable Virtual Machine Platform</summary>
			Enter Windows
			Search for Windows Features
			Check the box beside Virtual Machine Platform
			Press OK and restart the PC
		</details>
</details>
<details>
</summary>####Install WSL 2 on Windows</summary>
		Open Terminal (Search Terminal > right-click > Select Run as administrator)
		Enter `wsl --install` command*	
		Restart Computer
		<details>
		</summary>Note</summary>
		**The above command only works if WSL is not installed at all. If you run `wsl --install` and see 			the WSL help text, please try running `wsl --list --online` to see a list of available distros and 			run `wsl --install -d[DistroName]` to install a distro. If the install process hangs at 0.0%, run 			`wsl --install --web-download -d[DistroName]` to first download the distribution prior to installing.*
        </details>
</details>
</details>
##Change the Linux distribution Installed (ArchLinux)
<details>
<summary>Expand for details</summary>
  		<details>
        <summary>Open PowerShell</summary>
            In windows, Search PowerShell > right-click > Select Run as administrator
        Enter `wsl --install archlinux` command
        </details>
        <details>
        <summary>Upgrade installed WSL version to 2</summary>
            In powershell, Enter `wsl --set-version archlinux 2` command
        </details>
</details>
##Setting up ArchLinux
<details>
<summary>Expand for details</summary>
    <details>
    <summary>Opening WSL 2</summary>
        Run Terminal as administrator
        Enter `wsl` command
        <details>
        <summary>Common WSL Commands</summary>
            `wsl --update`
            `wsl --status`
            `wsl --wsl --set-version [DistroName] [Vers]`
            `wsl --version`
            `wsl --shutdown`
            `wsl --unregister [DistroName]
            *for more commands enter `wsl --help`
        </details>
    </details>
    <details>
        <summary>Update Nvidia GPU Driver</summary>
        Open Nvidia app
        Select Drivers tab
        Download and install latest Nvidia Studio Driver
        Restart Windows
    </details>
    <details>
        <summary>Add functions library</summary>
        Open Terminal > enter `wsl`
        Enter `pacman -Syu` command
    </details>
    <details>
        <summary>Set password for root user</summary>
        	Enter `passwd root` command
        	Enter desired password twice
    </details>
    <details>
        <summary>Add new user and set new user password</summary>
        Enter `cd` 
        `useradd [username]`
        `passwd [username]`
        Enter desired password twice
    </details>
    <details>
        <summary>Install Nano</summary>
        Enter `pacman -S nano`
    </details>
    <details>
        <summary>Install Sudo</summary>
        Enter `pacman -S sudo`
    </details>
    <details>
        <summary>Set default user</summary>
        Ensure user has been created
        <details>
            <summary>Append the `/etc/wsl.conf`file</summary>
            `nano /etc/wsl.conf`
            Add 
            `[User]
            default=[username]`
        </details>
        Exit and Shutdown WSL
    </details>
    <details>
        <summary>Add user to sudo group</summary>
        <details>
            <summary>Login using root user</summary>
            `wsl -u root`
        </details>
        `nano /etc/sudoers`
        Before `root root ALL=(ALL:ALL) ALL` add line `[username] ALL= (ALL:ALL) ALL`
        Exit and Shutdown WSL
    </details>
    <details>
        <summary>Install Git</summary>
        `sudo pacman -S git`
    </details>
    <details>
        <summary>Install CMake</summary>
        `sudo pacman -S cmake`
    </details>
    <details>
        <summary>Install WSL 2 CUDA</summary>
        `sudo pacman -S cuda`
        `sudo pacman -S base-devel`
        `git clone https://github.com/NVIDIA/cuda-samples.git`
        Restart WSL
        `cd cuda-samples/Samples/1_Utilities/deviceQuery`
        `mkdir build`
        `cd build`
        `cmake ..`
        `make`
        `./deviceQuery`
        If `Result=PASS` means success
    </details>
    <details>
        <summary>Install LXQT</summary>
        Enter WSL
        `sudo pacman -S lxqt`
        Exit and Shutdown WSL
        Enter WSL
        `startlxqt`
    </details>
</details>
##Install PyTorch
<details>
    <summary>Expand for details</summary>
    <details>
        <summary>Install Python3.13</summary>
        `sudo pacman -S --needed base-devel git` 
        `git clone https://aur.archlinux.org/python313.git` 
        `cd python313`
        `makepkg -si`
    </details>
</details>
