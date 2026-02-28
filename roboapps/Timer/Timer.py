# APP_NAME: Timer
import tkinter as tk
from datetime import datetime

def create_round_button(parent, text, command, color, hover_color):
    canvas = tk.Canvas(parent, width=100, height=100, bg="#1e1e2e", highlightthickness=0)
    
    def draw_button(fill):
        canvas.delete("all")
        canvas.create_oval(5, 5, 95, 95, fill=fill, outline="white", width=2)
        canvas.create_text(50, 50, text=text, fill="white", font=("Helvetica", 14, "bold"))
    
    draw_button(color)
    
    def on_enter(e):
        draw_button(hover_color)
    def on_leave(e):
        draw_button(color)
    def on_click(e):
        command()

    canvas.bind("<Enter>", on_enter)
    canvas.bind("<Leave>", on_leave)
    canvas.bind("<Button-1>", on_click)
    
    return canvas

def on_start():
    now = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    start_label.config(text=f"▶  Start Time:   {now}")

def on_stop():
    now = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    stop_label.config(text=f"⏹  Stop Time:   {now}")

# --- Main Window ---
root = tk.Tk()
root.title("Timer")
root.geometry("420x280")
root.resizable(False, False)
root.configure(bg="#1e1e2e")

# --- Title ---
title = tk.Label(root, text="Timer", font=("Helvetica", 20, "bold"),
                 bg="#1e1e2e", fg="white")
title.pack(pady=(20, 10))

# --- Buttons Row ---
btn_frame = tk.Frame(root, bg="#1e1e2e")
btn_frame.pack(pady=10)

start_btn = create_round_button(btn_frame, "Start", on_start,
                                 color="#27ae60", hover_color="#2ecc71")
start_btn.grid(row=0, column=0, padx=30)

stop_btn = create_round_button(btn_frame, "Stop", on_stop,
                                color="#c0392b", hover_color="#e74c3c")
stop_btn.grid(row=0, column=1, padx=30)

# --- Time Labels ---
info_frame = tk.Frame(root, bg="#1e1e2e")
info_frame.pack(pady=20)

start_label = tk.Label(info_frame, text="▶  Start Time:   --:--:--",
                       font=("Helvetica", 13), bg="#1e1e2e", fg="#2ecc71",
                       anchor="w", width=32)
start_label.pack(anchor="w", pady=4)

stop_label = tk.Label(info_frame, text="⏹  Stop Time:   --:--:--",
                      font=("Helvetica", 13), bg="#1e1e2e", fg="#e74c3c",
                      anchor="w", width=32)
stop_label.pack(anchor="w", pady=4)

root.mainloop()
