# Installing WSL2/ArchLinux/PyTorch

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

Refer to device's firmware documentation

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

**The above command only works if WSL is not installed at all. If you run**

```bash
wsl --install
```

**and see the WSL help text, please try running**

```bash
wsl --list --online
```

**to see a list of available distros and run**

```bash
wsl --install -d[DistroName]
```

**to install a distro. If the install process hangs at 0.0%, run**

```bash
wsl --install --web-download -d[DistroName]
```

**to first download the distribution prior to installing.*  

</details>

</details>

## Change the Linux distribution Installed (ArchLinux)

<details>
<summary><strong>Open PowerShell</strong></summary>

In windows, Search PowerShell > right-click > Select Run as administrator

```bash
wsl --install archlinux
```

</details>

<details>
<summary><strong>Upgrade installed WSL version to 2</strong></summary>

In powershell, Enter command

```bash
wsl --set-version archlinux 2
```

</details>

</details>

## Setting up ArchLinux

<br>

<details>
<summary><strong>Expand for details</strong></summary>

<br>

<details>
<summary><strong>Opening WSL 2</strong></summary>

1. Run Terminal as administrator  
2. Enter command  

```bash
wsl
```

<details>
<summary><strong>Common WSL Commands</strong></summary>

```bash
wsl --update
wsl --status
wsl --wsl --set-version [DistroName] [Vers]
wsl --version
wsl --shutdown
wsl --unregister [DistroName]
```

*for more commands enter*

```bash
wsl --help
```

</details>

</details>

<details>
<summary><strong>Update Nvidia GPU Driver</strong></summary>

1. Open Nvidia app  
2. Select Drivers tab  
3. Download and install latest Nvidia Studio Driver  
4. Restart Windows  

</details>

<details>
<summary><strong>Add functions library</strong></summary>

1. Open Terminal > enter  

```bash
wsl
```

2. Enter command  

```bash
pacman -Syu
```

</details>

<details>
<summary><strong>Set password for root user</strong></summary>

1. Enter command  

```bash
passwd root
```

2. Enter desired password twice  

</details>

<details>
<summary><strong>Add new user and set new user password</strong></summary>

```bash
cd
useradd -m [username]
passwd [username]
```

Enter desired password twice

</details>

<details>
<summary><strong>Install Nano</strong></summary>

```bash
pacman -S nano
```

</details>

<details>
<summary><strong>Install Sudo</strong></summary>

```bash
pacman -S sudo
```

</details>

<details>
<summary><strong>Set default user</strong></summary>

Ensure user has been created

<details>
<summary><strong>Append the <code>/etc/wsl.conf</code> file</strong></summary>

```bash
nano /etc/wsl.conf
```

Add

```ini
[User]
default=[username]
```

</details>

Exit and Shutdown WSL

</details>

<details>
<summary><strong>Append the .bashrc file</strong></summary>

```bash
nano ~/.bashrc
```

Add

```bash
PS1="\[\e[37;40m\][\[\e[32;40m\]\u\[\e[37;40m\]@\h \[\e[36;40m\]\w\[\e[0m\]]\\\$ "
export QT_SCALE_FACTOR=2
cd ~
```

Exit and shutdown WSL

</details>

<details>
<summary><strong>Add user to sudo group</strong></summary>

<details>
<summary><strong>Login using root user</strong></summary>

```bash
wsl -u root
```

</details>

```bash
nano /etc/sudoers
```

Before

```text
root root ALL=(ALL:ALL) ALL
```

add line

```text
[username] ALL= (ALL:ALL) ALL
```

Exit and Shutdown WSL

</details>

<details>
<summary><strong>Install Git</strong></summary>

```bash
sudo pacman -S git
```

</details>

<details>
<summary><strong>Install CMake</strong></summary>

```bash
sudo pacman -S cmake
```

</details>

<details>
<summary><strong>Install WSL 2 CUDA</strong></summary>

```bash
sudo pacman -S cuda
sudo pacman -S base-devel
git clone https://github.com/NVIDIA/cuda-samples.git
```

Restart WSL

```bash
cd cuda-samples/Samples/1_Utilities/deviceQuery
mkdir build
cd build
cmake ..
make
./deviceQuery
```

If `Result=PASS` means success

</details>

<details>
<summary><strong>Install LXQT</strong></summary>

1. Enter WSL  

```bash
sudo pacman -S lxqt
```

2. Exit and Shutdown WSL  
3. Enter WSL  

```bash
startlxqt
```

</details>

## Install PyTorch

<br>

<details>
<summary><strong>Install Python3.13</strong></summary>

```bash
sudo pacman -S --needed base-devel git
git clone https://aur.archlinux.org/python313.git
cd python313
makepkg -si
```

</details>

<details>
<summary><strong>Set up python venv</strong></summary>

```bash
~cd
python3.13 -m venv venv
source ~/venv/bin/activate
```

</details>

<details>
<summary><strong>Set up PyTorch</strong></summary>

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu130
```

</details>

## Set up Text to Speech (TTS)

<br>

<details>
<summary><strong>Install Speechbrain in venv</strong></summary>

```bash
git clone https://github.com/speechbrain/speechbrain.git
cd speechbrain
pip install -r requirements.txt
pip install --editable .
```

Exit and Restart WSL

</details>

<details>
<summary><strong>Install TorchAudio</strong></summary>

Enter venv

```bash
source ~/venv/bin/activate
pip install torchaudio --index-url https://download.pytorch.org/whl/cu130
```

</details>

<details>
<summary><strong>Install TorchCodec</strong></summary>

```bash
pip install torchcodec --index-url https://download.pytorch.org/whl/cu130
```

</details>

<details>
<summary><strong>Perform TTS with FastSpeech2</strong></summary>

Enter venv

```bash
source ~/venv/bin/activate
mkdir speechbrain-test
cd speechbrain-test
nano fastspeech2.py
```

Write to file

```python
import torchaudio
from speechbrain.inference.TTS import FastSpeech2
from speechbrain.inference.vocoders import HIFIGAN

# Intialize TTS (tacotron2) and Vocoder (HiFIGAN)
fastspeech2 = FastSpeech2.from_hparams(source="speechbrain/tts-fastspeech2-ljspeech", savedir="pretrained_models/tts-fastspeech2-ljspeech")
hifi_gan = HIFIGAN.from_hparams(source="speechbrain/tts-hifigan-ljspeech", savedir="pretrained_models/tts-hifigan-ljspeech")

# Run TTS with text input
input_text = "were the leaders in this luckless change; though our own Baskerville; who was at work some years before them; went much on the same lines;"

mel_output, durations, pitch, energy = fastspeech2.encode_text(
    [input_text],
    pace=1.0,        # scale up/down the speed
    pitch_rate=1.0,  # scale up/down the pitch
    energy_rate=1.0, # scale up/down the energy
)

# Running Vocoder (spectrogram-to-waveform)
waveforms = hifi_gan.decode_batch(mel_output)

# Save the waverform
torchaudio.save('example_TTS_input_text.wav', waveforms.squeeze(1), 22050)

# Run TTS with phoneme input
input_phonemes = ['W', 'ER', 'DH', 'AH', 'L', 'IY', 'D', 'ER', 'Z', 'IH', 'N', 'DH', 'IH', 'S', 'L', 'AH', 'K', 'L', 'AH', 'S', 'CH', 'EY', 'N', 'JH', 'spn', 'DH', 'OW', 'AW', 'ER', 'OW', 'N', 'B', 'AE', 'S', 'K', 'ER', 'V', 'IH', 'L', 'spn', 'HH', 'UW', 'W', 'AA', 'Z', 'AE', 'T', 'W', 'ER', 'K', 'S', 'AH', 'M', 'Y', 'IH', 'R', 'Z', 'B', 'IH', 'F', 'AO', 'R', 'DH', 'EH', 'M', 'spn', 'W', 'EH', 'N', 'T', 'M', 'AH', 'CH', 'AA', 'N', 'DH', 'AH', 'S', 'EY', 'M', 'L', 'AY', 'N', 'Z', 'spn']
mel_output, durations, pitch, energy = fastspeech2.encode_phoneme(
    [input_phonemes],
    pace=1.0,        # scale up/down the speed
    pitch_rate=1.0,  # scale up/down the pitch
    energy_rate=1.0, # scale up/down the energy
)

# Running Vocoder (spectrogram-to-waveform)
waveforms = hifi_gan.decode_batch(mel_output)

# Save the waverform
torchaudio.save('example_TTS_input_phoneme.wav', waveforms.squeeze(1), 22050)
```

Exit File

```bash
python3
```

```bash
```

</details>
