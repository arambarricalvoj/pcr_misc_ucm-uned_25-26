import pandas as pd
import matplotlib.pyplot as plt
import os
import io


def detect_separator(header_line):
    if "\t" in header_line:
        return "\t"
    elif ";" in header_line:
        return ";"
    elif "," in header_line:
        return ","
    else:
        return r"\s+"


def load_csv_with_footer(path):
    with open(path, "r") as f:
        lines = f.readlines()

    split_idx = None
    for i, line in enumerate(lines):
        if line.startswith("Metric"):
            split_idx = i
            break

    if split_idx is None:
        raise ValueError(f"No se encontró bloque de métricas en {path}")

    data_lines = lines[:split_idx]
    sep = detect_separator(data_lines[0])

    df = pd.read_csv(
        io.StringIO("".join(data_lines)),
        sep=sep,
        engine="python"
    )

    df.columns = [c.strip() for c in df.columns]

    if "time" not in df.columns:
        raise KeyError(f"'time' no está en columnas: {df.columns}")

    return df


def plot_csv(path, out_dir):
    filename = os.path.basename(path)
    name = os.path.splitext(filename)[0]

    df = load_csv_with_footer(path)

    # ===== Señales que queremos =====
    required_cols = ["xr", "yr", "d", "e_theta", "v", "w"]
    for col in required_cols:
        if col not in df.columns:
            raise KeyError(f"Falta columna '{col}' en {path}")

    time = df["time"]

    # =========================================
    # 1️⃣ Trayectoria xr vs yr
    # =========================================
    plt.figure(figsize=(6, 6))
    plt.plot(df["xr"], df["yr"], label="Trayectoria", linewidth=2.5)
    plt.scatter(df["xr"].iloc[-1], df["yr"].iloc[-1], color="red", label="Punto destino")
    plt.scatter(df["xr"].iloc[0], df["yr"].iloc[0], color="lightgreen", label="Punto inicio")
    plt.xlabel("Posición del robot en el eje X, xr (m)", fontsize=14, fontweight="bold")
    plt.ylabel("Posición del robot en el eje Y, yr (m)", fontsize=14, fontweight="bold")
    #plt.title(f"Trayectoria – {name}")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()

    traj_path = os.path.join(out_dir, f"{name}_trayectoria.png")
    plt.savefig(traj_path)
    plt.close()

    # =========================================
    # 2️⃣ Señales vs tiempo (SIN xr, yr)
    # =========================================
    plt.figure(figsize=(10, 6))

    time_signals = ["d", "e_theta", "v", "w"]

    for col in time_signals:
        plt.plot(time, df[col], label=col, linewidth=2.5)

    plt.xlabel("Tiempo (s)", fontsize=14, fontweight="bold")
    plt.ylabel("d (m), e_theta (rad), v (m/s), w (rad/s)", fontsize=14, fontweight="bold")
    #plt.title(f"Señales de control vs Tiempo – {name}",
    #        fontsize=16, fontweight="bold")

    plt.grid(True)
    plt.legend(fontsize=12)

    time_path = os.path.join(out_dir, f"{name}_tiempo.png")
    plt.savefig(time_path)
    plt.close()

    print(f"✔ Gráficas generadas: {traj_path} y {time_path}")


def main():
    out_dir = "graf"
    os.makedirs(out_dir, exist_ok=True)

    for file in os.listdir("."):
        if file.endswith(".csv"):
            plot_csv(file, out_dir)


if __name__ == "__main__":
    main()