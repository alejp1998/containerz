# LenovoLegionLinux Installation Status

**Date:** December 15, 2025  
**Laptop:** Lenovo Legion Pro 7 16IAX10H (Model 83F5)  
**BIOS:** Q7CN40WW

## Current Status: ⚠️ BLOCKED BY SECURE BOOT

### What's Been Done

✅ **Step 1: Dependencies Installed**
- All required packages installed successfully
- gcc, kernel headers, python3-pyqt6, dkms, etc.

✅ **Step 2: Repository Cloned**
- Location: `~/LenovoLegionLinux`
- Latest version from GitHub

✅ **Step 3: Module Compiled**
- Build successful in `~/LenovoLegionLinux/kernel_module`
- Module file created: `legion-laptop.ko`
- Warnings during build are normal

❌ **Step 4: Module Loading FAILED**
- Secure Boot is enabled
- Kernel refuses to load unsigned module
- Error: `Key was rejected by service`

### Why It's Blocked

Linux Secure Boot only allows loading **signed kernel modules**. The LenovoLegionLinux module is not signed by a trusted authority, so the kernel rejects it.

### Solutions

#### Option 1: Disable Secure Boot (Easiest - Recommended for Testing)

**Steps:**
1. Reboot your laptop
2. During boot, press **Delete** or **F2** to enter BIOS Setup
3. Navigate to **Security** tab
4. Find **Secure Boot** setting
5. Change to **Disabled**
6. Press **F10** to save and exit
7. Boot back into Linux

**Then run:**
```bash
cd ~/LenovoLegionLinux/kernel_module
sudo make reloadmodule
```

**Note:** You can re-enable Secure Boot later after testing.

#### Option 2: Sign the Kernel Module (Complex - Permanent Solution)

If you want to keep Secure Boot enabled:

```bash
# Follow DKMS signing instructions
# https://github.com/dell/dkms#secure-boot

# Summary:
# 1. Generate signing key
# 2. Enroll key in MOK (Machine Owner Key)
# 3. Sign the module
# 4. Reboot and confirm key enrollment
```

This is more complex but allows keeping Secure Boot enabled.

### What to Test After Loading Successfully

Once the module loads, test these in order:

#### 1. Verify Module Loaded
```bash
sudo dmesg | grep -i legion
```

**Expected:**
- Message about "legion_laptop loaded for this device"
- No error messages

**Possible Issues:**
- "not in allowlist" → Your model is too new (need to force load)
- "not loaded for this device" → Model not compatible

#### 2. Check Fan Curve Access
```bash
sudo cat /sys/kernel/debug/legion/fancurve
```

**Expected Output:**
```
EC Chip ID: 8227
EC Chip Version: [some value]
fan curve current point id: 0
fan curve points size: 8
[Fan curve table with values]
```

**If this works:** Module has successfully accessed the Embedded Controller!

#### 3. Check Available Power Modes
```bash
cat /sys/firmware/acpi/platform_profile_choices
```

**Expected:**
```
quiet balanced performance balanced-performance
```

**If you see `balanced-performance`:** Custom mode is available! This is the key to TDP control.

#### 4. Test Custom Power Mode
```bash
# Switch to custom mode
echo balanced-performance | sudo tee /sys/firmware/acpi/platform_profile

# Verify it changed
cat /sys/firmware/acpi/platform_profile

# Check if GPU power limit changed
nvidia-smi -q -d POWER
```

**Look for:** Changes in "Current Power Limit" (might increase from 95W)

#### 5. Configure GPU TDP (If Available)

The daemon (`legiond`) can control GPU TDP. Configuration file would be at `/etc/legion_linux/legiond.ini`.

Example configuration:
```ini
[gpu_control]
gpu_control = nvidia
tdp_ac_p = 150        # Performance mode on AC: 150W
tdp_ac_bp = 130       # Custom mode on AC: 130W
tdp_ac_b = 100        # Balanced mode on AC: 100W
tdp_bat_bp = 80       # Custom mode on battery: 80W
```

**Note:** This requires daemon installation and may not work on all models.

### Expected Outcomes

#### Best Case ✅
- Module loads without errors
- Fan curve accessible
- Custom mode available (`balanced-performance`)
- GPU TDP can be controlled via daemon
- Power limit increases to 115-175W

#### Good Case ⚠️
- Module loads (maybe with `force=1`)
- Fan curve accessible
- Power modes work
- GPU TDP control **doesn't work** (VBIOS still blocks it)
- You get excellent fan control but same 95W limit

#### Worst Case ❌
- Module loads but shows "not in allowlist"
- Model is too new, EC firmware different
- No features work
- Need to wait for maintainer to add support

### If Your Model Isn't Supported

**Report it to the maintainer:**
1. Go to https://github.com/johnfanv2/LenovoLegionLinux/issues
2. Create new issue with title: "Support Request: Legion Pro 7 16IAX10H (83F5)"
3. Include:
   - Model: 83F5 (Legion Pro 7 16IAX10H)
   - BIOS: Q7CN40WW
   - GPU: RTX 5090 Laptop
   - What works / doesn't work
   - Output of `sudo dmesg | grep legion`
   - Output of `sudo dmidecode -t system`

The maintainer may add support or provide a test build for Gen 10 models.

### Current Reality

Your RTX 5090 Laptop GPU has a **95W VBIOS power limit**. Even if LenovoLegionLinux works:

**What it CAN do:**
- Control fan curves precisely
- Switch power modes via software
- Enable battery conservation mode
- Potentially access custom mode

**What it MIGHT NOT do:**
- Bypass the 95W VBIOS limit
- Increase GPU power beyond what BIOS allows
- Override hardware restrictions

**Why:** The 95W limit is enforced at multiple levels:
1. VBIOS firmware (blocks nvidia-smi)
2. Embedded Controller (might be controlled by LenovoLegionLinux)
3. Motherboard VRM limits (physical hardware)
4. BIOS power management (might be adjustable)

LenovoLegionLinux accesses level 2 (EC), which *could* help, but levels 1, 3, and 4 may still limit you to 95W.

### Recommendation

1. **Test it anyway** - Disable Secure Boot and see what happens
2. **Expect fan control** to work well
3. **Don't expect** the 95W limit to magically disappear
4. **Report results** to help the community
5. **Consider Windows testing** - Boot Windows, install Lenovo Vantage, check if it offers higher TDP modes

### Quick Commands Reference

```bash
# After disabling Secure Boot:

# Load module
cd ~/LenovoLegionLinux/kernel_module
sudo make reloadmodule

# Check status
sudo dmesg | tail -30
sudo cat /sys/kernel/debug/legion/fancurve

# Test power modes
cat /sys/firmware/acpi/platform_profile_choices
echo balanced-performance | sudo tee /sys/firmware/acpi/platform_profile

# Monitor GPU
watch -n 1 nvidia-smi

# Run benchmark again
cd ~/dev/containerz/llama-cpp
./benchmark-gpu-performance.sh
```

### Files Created

- `~/LenovoLegionLinux/` - Full repository
- `~/LenovoLegionLinux/kernel_module/legion-laptop.ko` - Compiled kernel module
- `/home/alejp/dev/containerz/RTX-5090-POWER-ANALYSIS.md` - Complete analysis
- `/home/alejp/dev/containerz/LENOVOLEGIONLINUX-STATUS.md` - This file

---

**Next Action:** Disable Secure Boot in BIOS and retry module loading to see if your Gen 10 Legion is supported.
