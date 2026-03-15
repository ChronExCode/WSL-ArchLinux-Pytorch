# **Installing WSL2/ArchLinux/PyTorch**

------



```html
## Install WSL 2
<details>
  <summary>Expand for details</summary>

  <details>
    <summary>Enable Virtualization on Windows (Enabled by default on Windows manufactured PCs)</summary>

    <details>
      <summary>Access the UEFI/BIOS</summary>
      Open Settings app &gt; System &gt; Recovery<br>
      Select Recovery Options &gt; Advanced Startup &gt; Restart Now<br>
      Choose Troubleshoot &gt; Advanced options &gt; UEFI Firmware Settings &gt; Restart<br>
      UEFI utility will open after restart
    </details>

    <details>
      <summary>Make changes in the UEFI/BIOS (Based on Manufacturer)</summary>

      <details>
        <summary>Acer (Intel processor)</summary>
        Select Advanced Page in BIOS<br>
        Enable the Intel VTX and Intel VTD settings (On certain devices press Ctrl+S first)<br>
        Press F10
      </details>

      <details>
        <summary>Acer (AMD processor)</summary>
        Select Advanced Page in BIOS<br>
        Enable the AMD-SVM and AMD-IOMMU settings (On certain devices press Ctrl+S first)<br>
        Press F10
      </details>

      <details>
        <summary>ASUS (AMD Notebook)</summary>
        Select Advanced Page in BIOS<br>
        Enable SVM mode<br>
        Press F10
      </details>

      <details>
        <summary>ASUS (Intel Notebook)</summary>
        Select Advanced Page in BIOS<br>
        Enable Virtualization Technology mode<br>
        Press F10
      </details>

      <details>
        <summary>ASUS (Intel Motherboard)</summary>
        Press F7 to enter Advanced Mode (ROG motherboards enter directly, no need to press F7)<br>
        Select Advanced Page &gt; CPU Configuration<br>
        Enable Intel VMX Virtualization Technology (Enabled by default)
      </details>

      <details>
        <summary>Dell</summary>
        Select Advanced page / Virtualization Support in BIOS &gt; Virtualization<br>
        Enable Virtualization<br>
        Set Virtualization for Direct-IO or VT-d to Enabled<br>
        Press F10
      </details>

      <details>
        <summary>HP (OMEN by HP, Victus by HP, HP Spectre, HP ENVY, and HP Pavilion series)</summary>
        Select Configuration in BIOS settings &gt; Virtualization Technology<br>
        Enable Virtualization Technology<br>
        Press F10
      </details>

      <details>
        <summary>HP (HP EliteBook, HP EliteDesk, HP ZHAN Notebook series)</summary>
        Select Advanced page in BIOS &gt; System Options &gt; press Enter<br>
        Enable Virtualization Technology (VTx)<br>
        Press F10
      </details>

      <details>
        <summary>HP (HP workstations)</summary>
        Select Security tab in BIOS &gt; USB Security &gt; press Enter<br>
        Enable Virtualization Technology (VTx)<br>
        Press F10
      </details>

      <details>
        <summary>Lenovo (ThinkPad)</summary>
        Select Security page in BIOS<br>
        Enable Intel(R) Virtualization Technology and Intel VT-d Feature<br>
        Press F10
      </details>

      <details>
        <summary>Lenovo (IdeaPad)</summary>
        Select Configuration page in BIOS<br>
        Enable AMD V(TM) Technology<br>
        Press F10
      </details>

      <details>
        <summary>Lenovo (ThinkCentre)</summary>
        Select Advanced page in BIOS<br>
        Enable Intel(R) Virtualization Technology<br>
        Press F10
      </details>

      <details>
        <summary>Unlisted Manufacturers</summary>
        Refer to the device's firmware documentation
      </details>
    </details>

    <details>
      <summary>Enable Virtual Machine Platform</summary>
      Enter Windows<br>
      Search for Windows Features<br>
      Check the box beside Virtual Machine Platform<br>
      Press OK and restart the PC
    </details>
  </details>

  <details>
    <summary>Install WSL 2 on Windows</summary>
    Open Terminal (Search Terminal &gt; right-click &gt; Select Run as administrator)<br>
    Enter <code>wsl --install</code> command*<br>
    Restart Computer

    <details>
      <summary>Note</summary>
      <strong>The above command only works if WSL is not installed at all.</strong>
      If you run <code>wsl --install</code> and see the WSL help text, try running
      <code>wsl --list --online</code> to see a list of available distros, then run
      <code>wsl --install -d [DistroName]</code> to install a distro.
      If the install process hangs at 0.0%, run
      <code>wsl --install --web-download -d [DistroName]</code> to first download the distribution prior to installing.*
    </details>
  </details>
</details>

## Change the Linux distribution installed (Arch Linux)
<details>
  <summary>Expand for details</summary>

  <details>
    <summary>Open PowerShell</summary>
    In Windows, search PowerShell &gt; right-click &gt; Select Run as administrator<br>
    Enter <code>wsl --install archlinux</code> command
  </details>

  <details>
    <summary>Upgrade installed WSL version to 2</summary>
    In PowerShell, enter <code>wsl --set-version archlinux 2</code> command
  </details>
</details>

## Setting up Arch Linux
<details>
  <summary>Expand for details</summary>

  <details>
    <summary>Opening WSL 2</summary>
    Run Terminal as administrator<br>
    Enter <code>wsl</code> command

    <details>
      <summary>Common WSL Commands</summary>
      <code>wsl --update</code><br>
      <code>wsl --status</code><br>
      <code>wsl --set-version [DistroName] [Vers]</code><br>
      <code>wsl --version</code><br>
      <code>wsl --shutdown</code><br>
      <code>wsl --unregister [DistroName]</code><br>
      <em>For more commands, enter <code>wsl --help</code></em>
    </details>
  </details>

  <details>
    <summary>Update Nvidia GPU Driver</summary>
    Open Nvidia app<br>
    Select Drivers tab<br>
    Download and install latest Nvidia Studio Driver<br>
    Restart Windows
  </details>

  <details>
    <summary>Add functions library</summary>
    Open Terminal &gt; enter <code>wsl</code><br>
    Enter <code>pacman -Syu</code> command
  </details>

  <details>
    <summary>Set password for root user</summary>
    Enter <code>passwd root</code> command<br>
    Enter desired password twice
  </details>

  <details>
    <summary>Add new user and set new user password</summary>
    Enter <code>cd</code><br>
    <code>useradd [username]</code><br>
    <code>passwd [username]</code><br>
    Enter desired password twice
  </details>

  <details>
    <summary>Install Nano</summary>
    Enter <code>pacman -S nano</code>
  </details>

  <details>
    <summary>Install Sudo</summary>
    Enter <code>pacman -S sudo</code>
  </details>

  <details>
    <summary>Set default user</summary>
    Ensure user has been created

    <details>
      <summary>Append the /etc/wsl.conf file</summary>
      <code>nano /etc/wsl.conf</code><br>
      Add:<br>
      <pre>[user]
default=[username]</pre>
    </details>

    Exit and shut down WSL
  </details>

  <details>
    <summary>Add user to sudo group</summary>

    <details>
      <summary>Login using root user</summary>
      <code>wsl -u root</code>
    </details>

    <code>nano /etc/sudoers</code><br>
    Before <code>root ALL=(ALL:ALL) ALL</code> add line
    <code>[username] ALL=(ALL:ALL) ALL</code><br>
    Exit and shut down WSL
  </details>

  <details>
    <summary>Install Git</summary>
    <code>sudo pacman -S git</code>
  </details>

  <details>
    <summary>Install CMake</summary>
    <code>sudo pacman -S cmake</code>
  </details>

  <details>
    <summary>Install WSL 2 CUDA</summary>
    <code>sudo pacman -S cuda</code><br>
    <code>sudo pacman -S base-devel</code><br>
    <code>git clone https://github.com/NVIDIA/cuda-samples.git</code><br>
    Restart WSL<br>
    <code>cd cuda-samples/Samples/1_Utilities/deviceQuery</code><br>
    <code>mkdir build</code><br>
    <code>cd build</code><br>
    <code>cmake ..</code><br>
    <code>make</code><br>
    <code>./deviceQuery</code><br>
    If <code>Result=PASS</code>, installation succeeded
  </details>

  <details>
    <summary>Install LXQt</summary>
    Enter WSL<br>
    <code>sudo pacman -S lxqt</code><br>
    Exit and shut down WSL<br>
    Enter WSL<br>
    <code>startlxqt</code>
  </details>
</details>

## Install PyTorch
<details>
  <summary>Expand for details</summary>

  <details>
    <summary>Install PyTorch in Arch Linux on WSL</summary>
  </details>
</details>
```

​	