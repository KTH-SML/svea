# LIDAR Setup

To permanently set a static IP via GUI on **Ubuntu Desktop** (for your Hokuyo LiDAR setup), follow these steps:

---

## ✅ Step 1: Open Network Settings

1. Click the **network icon** in the top-right corner of the screen.  
2. Choose **"Wired Settings"** (or **"Settings → Network"**).  
3. Click the **gear ⚙️ icon** next to your Ethernet connection (e.g., *"Wired connection 1"*).

---

## 🌐 Step 2: Configure IPv4

1. Go to the **"IPv4"** tab.  
2. Set **Method** to `Manual`.  
3. Under **Addresses**, click **Add** and enter:  

   | Field     | Value         |
   |-----------|---------------|
   | Address   | `192.168.3.100` |
   | Netmask   | `255.255.255.0` |
   | Gateway   | *(optional)* leave blank or set to `192.168.3.1` if using a router |

4. **DNS**: You can leave it empty or use:  
```
8.8.8.8, 1.1.1.1
```

---

## 💾 Step 3: Save and Apply

1. Click **Apply**.  
2. Toggle the Ethernet connection **off and on** again.  
- Alternatively, restart your PC.  

---

## 🧪 Step 4: Verify

Open a terminal and run:

```bash
ip a
```

Check that your Ethernet interface (e.g.,
**enp3s0**) now shows **192.168.3.100**.
Then test:

```bash
ping 192.168.3.11
nc -zv 192.168.3.11 10940
```

Now your static IP is set permanently and the machine is ready to communicate with the Hokuyo LiDAR.
Let me know if you're using a specific Ubuntu version or a different desktop environment (e.g., KDE), and I can adjust the steps.
