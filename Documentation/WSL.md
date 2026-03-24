# Install WSL 2

<br>

## Enable Virtualization on Windows (Enabled by default on Windows manufactured PCs)
### Access the UEFI/BIOS

1. Open settings app > System > Recovery  
2. Select Recovery Options > Advanced Startup > Restart Now  
3. Choose Troubleshoot > Advanced options > UEFI Firmware Settings > Restart  
4. UEFI utility will open post restart  

### Make changes in the UEFI/BIOS (Based on Manufacturer)

<details>
<summary>Acer (Intel processor)</summary>

1. Select Advanced Page in BIOS  
2. Enable the Intel VTX and Intel VTD settings (On certain devices press Ctrl+S first)  
3. Press F10  

</details>

<details>
<summary>Acer (AMD processor)</summary>

1. Select Advanced Page in BIOS  
2. Enable the AMD-SVM and AMD-IOMMU settings (On certain devices press Ctrl+S first)  
3. Press F10  

</details>

<details>
<summary>ASUS (AMD Notebook)</summary>

1. Select Advanced Page in BIOS  
2. Enable SVM mode  
3. Press F10  

</details>

<details>
<summary>ASUS (Intel Notebook)</summary>

1. Select Advanced Page in BIOS  
2. Enable Virtualization Technology mode  
3. Press F10  

</details>

<details>
<summary>ASUS (Intel Motherboard)</summary>

1. Press F7 to enter Advanced Mode (ROG motherboards enter directly, no need to F7)  
2. Select Advanced Page > CPU Configuration  
3. Enable Intel VMX Virtualization Technology (Enabled by default)  

</details>

<details>
<summary>Dell</summary>

1. Select Advanced page/Virtualization Support in BIOS > Virtualization  
2. Enable Virtualization  
3. Set Virtualization for Direct-IO or VT-d to Enabled  
4. Press F10  

</details>

<details>
<summary>HP (OMEN by HP, Victus by HP, HP Spectre, HP ENVY, and HP Pavilion series)</summary>

1. Select Configuration in BIOS settings > Virtualization Technology  
2. Enable Virtualization Technology  
3. Press F10  

</details>

<details>
<summary>HP (HP EliteBook, HP EliteDesk, HP ZHAN Notebook series)</summary>

1. Select Advanced page in BIOS > System Options > press Enter  
2. Enable Virtualization Technology (VTx)  
3. Press F10  

</details>

<details>
<summary>HP (HP workstations)</summary>

1. Select Security tab in BIOS > USB Security > press Enter  
2. Enable Virtualization Technology (VTx)  
3. Press F10  

</details>

<details>
<summary>Lenovo (Thinkpad)</summary>

1. Select Security page in BIOS  
2. Enable Intel(R) Virtualization Technology and Intel VT-d Feature  
3. Press F10  

</details>

<details>
<summary>Lenovo (Ideapad)</summary>

1. Select Configuration page in BIOS  
2. Enable AMD V(TM) Technology  
3. Press F10  

</details>

<details>
<summary>Lenovo (ThinkCentre)</summary>

1. Select Advanced page in BIOS  
2. Enable Intel (R) Virtualization Technology  
3. Press F10  

</details>

<details>
<summary>Unlisted Manufacturers</summary>

**Refer to device's firmware documentation*

</details>

### Enable Virtual Machine Platform

1. Enter Windows  
2. Search for Windows Features  
3. Check the box beside Virtual Machine Platform  
4. Press OK and restart the PC  

## Install WSL 2 on Windows

1. Open Terminal (Search Terminal > right-click > Select Run as administrator)  
2. Enter command*  

```bash
wsl --install
```

3. Restart Computer  

<details>
<summary><em>*Note</em></summary>

*The above command only works if WSL is not installed at all.*
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

