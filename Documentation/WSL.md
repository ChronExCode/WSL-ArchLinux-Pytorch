## Install WSL 2

<br>

<details>
<summary><strong>Enable Virtualization on Windows (Enabled by default on Windows manufactured PCs)</strong></summary>

<br>

<details>
<summary><strong>Access the UEFI/BIOS</strong></summary>

1. Open settings app > System > Recovery  
2. Select Recovery Options > Advanced Startup > Restart Now  
3. Choose Troubleshoot > Advanced options > UEFI Firmware Settings > Restart  
4. UEFI utility will open post restart  

</details>

<details>
<summary><strong>Make changes in the UEFI/BIOS (Based on Manufacturer)</strong></summary>

<br>

<details>
<summary><strong>Acer (Intel processor)</strong></summary>

1. Select Advanced Page in BIOS  
2. Enable the Intel VTX and Intel VTD settings (On certain devices press Ctrl+S first)  
3. Press F10  

</details>

<details>
<summary><strong>Acer (AMD processor)</strong></summary>

1. Select Advanced Page in BIOS  
2. Enable the AMD-SVM and AMD-IOMMU settings (On certain devices press Ctrl+S first)  
3. Press F10  

</details>

<details>
<summary><strong>ASUS (AMD Notebook)</strong></summary>

1. Select Advanced Page in BIOS  
2. Enable SVM mode  
3. Press F10  

</details>

<details>
<summary><strong>ASUS (Intel Notebook)</strong></summary>

1. Select Advanced Page in BIOS  
2. Enable Virtualization Technology mode  
3. Press F10  

</details>

<details>
<summary><strong>ASUS (Intel Motherboard)</strong></summary>

1. Press F7 to enter Advanced Mode (ROG motherboards enter directly, no need to F7)  
2. Select Advanced Page > CPU Configuration  
3. Enable Intel VMX Virtualization Technology (Enabled by default)  

</details>

<details>
<summary><strong>Dell</strong></summary>

1. Select Advanced page/Virtualization Support in BIOS > Virtualization  
2. Enable Virtualization  
3. Set Virtualization for Direct-IO or VT-d to Enabled  
4. Press F10  

</details>

<details>
<summary><strong>HP (OMEN by HP, Victus by HP, HP Spectre, HP ENVY, and HP Pavilion series)</strong></summary>

1. Select Configuration in BIOS settings > Virtualization Technology  
2. Enable Virtualization Technology  
3. Press F10  

</details>

<details>
<summary><strong>HP (HP EliteBook, HP EliteDesk, HP ZHAN Notebook series)</strong></summary>

1. Select Advanced page in BIOS > System Options > press Enter  
2. Enable Virtualization Technology (VTx)  
3. Press F10  

</details>

<details>
<summary><strong>HP (HP workstations)</strong></summary>

1. Select Security tab in BIOS > USB Security > press Enter  
2. Enable Virtualization Technology (VTx)  
3. Press F10  

</details>

<details>
<summary><strong>Lenovo (Thinkpad)</strong></summary>

1. Select Security page in BIOS  
2. Enable Intel(R) Virtualization Technology and Intel VT-d Feature  
3. Press F10  

</details>

<details>
<summary><strong>Lenovo (Ideapad)</strong></summary>

1. Select Configuration page in BIOS  
2. Enable AMD V(TM) Technology  
3. Press F10  

</details>

<details>
<summary><strong>Lenovo (ThinkCentre)</strong></summary>

1. Select Advanced page in BIOS  
2. Enable Intel (R) Virtualization Technology  
3. Press F10  

</details>

<details>
<summary><strong>Unlisted Manufacturers</strong></summary>

**Refer to device's firmware documentation*

</details>

</details>

<details>
<summary><strong>Enable Virtual Machine Platform</strong></summary>

1. Enter Windows  
2. Search for Windows Features  
3. Check the box beside Virtual Machine Platform  
4. Press OK and restart the PC  

</details>

</details>

<details>
<summary><strong>Install WSL 2 on Windows</strong></summary>

1. Open Terminal (Search Terminal > right-click > Select Run as administrator)  
2. Enter command*  

```bash
wsl --install
```

3. Restart Computer  

<details>
<summary><strong>Note</strong></summary>

**The above command only works if WSL is not installed at all.*
```bash
If you run

wsl --install

and see the WSL help text, please try running

wsl --list --online

to see a list of available distros and run

wsl --install -d[DistroName]

to install a distro. If the install process hangs at 0.0%, run

wsl --install --web-download -d[DistroName]

to first download the distribution prior to installing.
```
</details>
</details>

