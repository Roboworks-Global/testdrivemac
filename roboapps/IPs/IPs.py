# APP_NAME: IPs
import tkinter as tk
from tkinter import ttk
import socket
import subprocess
import platform
import threading
import ipaddress

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None

def ping_host(ip):
    system = platform.system()
    if system == "Windows":
        cmd = ["ping", "-n", "1", "-w", "500", str(ip)]
    else:
        cmd = ["ping", "-c", "1", "-W", "1", str(ip)]
    try:
        result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception:
        return False

def get_hostname(ip):
    try:
        return socket.gethostbyaddr(str(ip))[0]
    except Exception:
        return "N/A"

def scan_subnet(local_ip, update_callback, done_callback):
    try:
        network = ipaddress.IPv4Network(f"{local_ip}/24", strict=False)
    except Exception:
        done_callback()
        return

    hosts = list(network.hosts())
    threads = []
    results = []
    lock = threading.Lock()

    def check(ip):
        if ping_host(ip):
            hostname = get_hostname(ip)
            with lock:
                results.append((str(ip), hostname))
            update_callback(str(ip), hostname)

    for ip in hosts:
        t = threading.Thread(target=check, args=(ip,), daemon=True)
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    done_callback()

def main():
    root = tk.Tk()
    root.title("IPs")
    root.geometry("520x400")
    root.resizable(True, True)

    # Header
    header = tk.Label(root, text="Available IPs in Subnet", font=("Helvetica", 14, "bold"), pady=10)
    header.pack()

    local_ip = get_local_ip()
    if local_ip:
        subnet_label = tk.Label(root, text=f"Local IP: {local_ip}  |  Scanning: {local_ip.rsplit('.', 1)[0]}.0/24",
                                font=("Helvetica", 10), fg="gray")
    else:
        subnet_label = tk.Label(root, text="Could not determine local IP.", font=("Helvetica", 10), fg="red")
    subnet_label.pack()

    # Table frame
    frame = tk.Frame(root)
    frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

    columns = ("IP Address", "Hostname")
    tree = ttk.Treeview(frame, columns=columns, show="headings", height=15)
    tree.heading("IP Address", text="IP Address")
    tree.heading("Hostname", text="Hostname")
    tree.column("IP Address", width=180, anchor="center")
    tree.column("Hostname", width=280, anchor="w")

    scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=tree.yview)
    tree.configure(yscrollcommand=scrollbar.set)
    tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # Status bar
    status_var = tk.StringVar(value="Scanning network...")
    status_label = tk.Label(root, textvariable=status_var, font=("Helvetica", 10), fg="blue", pady=5)
    status_label.pack()

    # Rescan button
    rescan_btn = tk.Button(root, text="Rescan", state=tk.DISABLED, font=("Helvetica", 11))
    rescan_btn.pack(pady=(0, 10))

    found_count = [0]

    def update_callback(ip, hostname):
        found_count[0] += 1
        root.after(0, lambda: tree.insert("", tk.END, values=(ip, hostname)))
        root.after(0, lambda: status_var.set(f"Scanning... {found_count[0]} host(s) found"))

    def done_callback():
        root.after(0, lambda: status_var.set(f"Scan complete. {found_count[0]} host(s) found."))
        root.after(0, lambda: rescan_btn.config(state=tk.NORMAL))

    def start_scan():
        for item in tree.get_children():
            tree.delete(item)
        found_count[0] = 0
        rescan_btn.config(state=tk.DISABLED)
        status_var.set("Scanning network...")
        if local_ip:
            t = threading.Thread(target=scan_subnet, args=(local_ip, update_callback, done_callback), daemon=True)
            t.start()
        else:
            status_var.set("No network connection found.")
            rescan_btn.config(state=tk.NORMAL)

    rescan_btn.config(command=start_scan)

    # Auto-start scan
    root.after(200, start_scan)
    root.mainloop()

if __name__ == "__main__":
    main()
