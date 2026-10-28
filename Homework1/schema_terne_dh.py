"""
Schema delle terne DH del braccio sinistro di Reachy 2 (Homework 1).

Disegna, nella posa di riferimento della Figura 3, le origini O_i e gli assi
x_i e z_i (y_i = z_i x x_i si ricava) per due tabelle DH equivalenti:

  A) x4 verso il polso  (convenzione DH "dal giunto i al giunto i+1")
     a4 = +0.28, alpha4 = +pi/2, alpha5 = -pi/2, theta = q + [0,0,0,pi,0,-pi,0]
  B) x4 verso la spalla (tabella adottata, coerente con il testo)
     a4 = -0.28, alpha4 = -pi/2, alpha5 = +pi/2, theta = q

Le direzioni degli assi NON sono disegnate a mano: sono calcolate con la catena
A_0_1 * ... * A_6_7 a partire dalla tabella DH, quindi lo schema e' esatto.
Le direzioni sono espresse nella terna 0 e disegnate in proiezione obliqua:
    x0 verso l'alto, z0 verso destra, y0 uscente dal foglio (diagonale).
Assi coincidenti sono uniti in un'unica freccia (es. "x0 = x1 = z2").
Le distanze tra le origini non sono in scala (le quote sono riportate a lato).

Sono disegnati anche gli angoli DH con frecce curve:
  theta_i (arancio): attorno a z_{i-1}, da x_{i-1} a x_i, in O'_i = O_{i-1} + d_i z_{i-1}
  alpha_i (verde):   attorno a x_i,     da z_{i-1} a z_i, in O_i
Gli archi seguono il verso reale della rotazione; un angolo nullo e' indicato con
un anello sulla punta dei due assi coincidenti. Gli assi di un'altra terna che
servono per un angolo sono riportati tratteggiati in grigio, tra parentesi.

Uso:  python3 schema_terne_dh.py      (richiede numpy e matplotlib)
Output: schema_terne_dh.png e schema_terne_dh.pdf nella cartella corrente.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

PI = np.pi

# --------------------------------------------------------------------------
# Dati
# --------------------------------------------------------------------------
Q_REF = np.array([0, -PI/2, -PI/2, -PI/2, 0, PI/2, 0])   # posa di riferimento (q_DH del testo)

# Tabelle DH: righe (a_i, d_i, alpha_i); theta_i = q_i + offset_i
SCHEMI = [
    dict(
        titolo=r"A)  $x_4$ verso il polso (convenzione DH)",
        sottotitolo=r"$a_4=+0.28$ m,  $\alpha_4=+\pi/2$,  $\alpha_5=-\pi/2$,  "
                    r"$\theta = q + [0,0,0,\pi,0,-\pi,0]$",
        tab=[(0, 0, -PI/2), (0, 0, -PI/2), (0, -0.28, -PI/2), (0.28, 0, PI/2),
             (0, 0, -PI/2), (0, 0, PI/2), (0, -0.11, PI)],
        offset=np.array([0, 0, 0, PI, 0, -PI, 0]),
    ),
    dict(
        titolo=r"B)  $x_4$ verso la spalla (tabella adottata)",
        sottotitolo=r"$a_4=-0.28$ m,  $\alpha_4=-\pi/2$,  $\alpha_5=+\pi/2$,  "
                    r"$\theta = q$",
        tab=[(0, 0, -PI/2), (0, 0, -PI/2), (0, -0.28, -PI/2), (-0.28, 0, -PI/2),
             (0, 0, PI/2), (0, 0, PI/2), (0, -0.11, PI)],
        offset=np.zeros(7),
    ),
]

# --------------------------------------------------------------------------
# Cinematica (terne espresse nella terna 0)
# --------------------------------------------------------------------------
def dh(a, d, theta, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0,   sa,     ca,    d   ],
                     [0,   0,      0,     1   ]])


def terne(tab, theta):
    T = np.eye(4)
    out = [T.copy()]                      # terna 0
    for (a, d, alpha), th in zip(tab, theta):
        T = T @ dh(a, d, th, alpha)
        out.append(T.copy())
    return out                            # terne 0..7


# --------------------------------------------------------------------------
# Proiezione obliqua: terna 0 -> piano del disegno
#   x0 -> su, z0 -> destra, y0 (uscente) -> diagonale in basso a sinistra
# --------------------------------------------------------------------------
P_X0 = np.array([0.0, 1.0])
P_Y0 = np.array([-0.55, -0.45])
P_Z0 = np.array([1.0, 0.0])


def proietta(v):
    return v[0] * P_X0 + v[1] * P_Y0 + v[2] * P_Z0


# Posizioni delle origini nel disegno (schematiche, non in scala)
Y_SPALLA, Y_GOMITO, Y_POLSO, Y_EE = 0.0, -2.6, -5.2, -7.1

def pos_disegno(p0):
    """Origine nel disegno a partire dalla quota lungo -x0 (in metri)."""
    s = -p0[0]                                     # distanza dalla spalla lungo il braccio
    if s < 1e-6:
        return np.array([0.0, Y_SPALLA])
    if s < 0.28 + 1e-6:
        return np.array([0.0, Y_GOMITO])
    if s < 0.56 + 1e-6:
        return np.array([0.0, Y_POLSO])
    return np.array([0.0, Y_EE])


# --------------------------------------------------------------------------
# Disegno
# --------------------------------------------------------------------------
L = 0.8                      # lunghezza frecce nel disegno
COL = {"x": "#d62728", "z": "#1f5fbf", "mix": "#7b3fa0"}


def freccia(ax, p, v, col, lw=2.0):
    ax.add_patch(FancyArrowPatch(p, p + v, arrowstyle="-|>", mutation_scale=14,
                                 color=col, lw=lw, zorder=4, shrinkA=0, shrinkB=0))


# --------------------------------------------------------------------------
# Angoli theta_i e alpha_i
#   theta_i: attorno a z_{i-1}, da x_{i-1} a x_i, disegnato in O'_i = O_{i-1} + d_i z_{i-1}
#   alpha_i: attorno a x_i,     da z_{i-1} a z_i, disegnato in O_i
# Gli archi sono calcolati in 3D (formula di Rodrigues) e proiettati, quindi
# mostrano il verso reale della rotazione. Un asse di un'altra terna necessario
# per un angolo e' riportato tratteggiato in grigio ("asse fantasma").
# --------------------------------------------------------------------------
COL_TH, COL_AL, COL_GH = "#e67e00", "#138d75", "0.45"

# raggio di ogni arco (diverso per archi che condividono gli stessi assi)
RAGGI = {
    ("a", 1): 0.38, ("t", 2): 0.42, ("a", 2): 0.38,
    ("t", 3): 0.34, ("a", 3): 0.42, ("t", 4): 0.58,
    ("a", 4): 0.30, ("a", 5): 0.62, ("t", 6): 0.44, ("a", 6): 0.44,
    ("a", 7): 0.44,
}
X_COL_DX, X_COL_SX, GAP_ETICH = 1.75, -1.85, 0.34    # colonne delle etichette degli angoli


def rodrigues(k, phi, v):
    k = k / np.linalg.norm(k)
    return v * np.cos(phi) + np.cross(k, v) * np.sin(phi) + k * np.dot(k, v) * (1 - np.cos(phi))


def fmt_ang(phi):
    m = ((int(round(phi / (PI / 2))) + 2) % 4) - 2
    return {0: "0", 1: r"\pi/2", -1: r"-\pi/2", -2: r"\pi"}[m]


def dir2d(v):
    u = proietta(v)
    return u / np.linalg.norm(u)


def arco(ax, p, v_start, k, phi, r, col):
    """Arco orientato da v_start ruotando di phi attorno a k; ritorna il punto medio."""
    ts = np.linspace(0.08, 0.92, 60)
    pts = np.array([p + r * dir2d(rodrigues(k, t * phi, v_start)) for t in ts])
    ax.plot(pts[:-3, 0], pts[:-3, 1], color=col, lw=1.6, zorder=3)
    ax.add_patch(FancyArrowPatch(pts[-5], pts[-1], arrowstyle="-|>", mutation_scale=11,
                                 color=col, lw=1.6, zorder=3, shrinkA=0, shrinkB=0))
    return pts[len(pts) // 2]


def anello(ax, tip, col):
    """Freccia ad anello attorno alla punta di due assi coincidenti (angolo nullo)."""
    r = 0.15
    a = np.linspace(0.3 * PI, 2.1 * PI, 50)
    pts = tip + r * np.c_[np.cos(a), np.sin(a)]
    ax.plot(pts[:-3, 0], pts[:-3, 1], color=col, lw=1.6, zorder=3)
    ax.add_patch(FancyArrowPatch(pts[-5], pts[-1], arrowstyle="-|>", mutation_scale=10,
                                 color=col, lw=1.6, zorder=3, shrinkA=0, shrinkB=0))
    return tip + np.array([-r, 0.0])                # punto di attacco della linea guida (a sinistra)


def piazza_etichette(ax, etichette):
    """Etichette in due colonne ai lati del braccio, distanziate, con linea guida verso l'arco."""
    for lato, x_col, ha in ((1, X_COL_DX, "left"), (-1, X_COL_SX, "right")):
        gruppo = sorted([e for e in etichette if e["lato"] == lato], key=lambda e: -e["pt"][1])
        y_prec = np.inf
        for e in gruppo:
            y = min(e["pt"][1], y_prec - GAP_ETICH)
            y_prec = y
            ax.plot([e["pt"][0], x_col - 0.06 * lato], [e["pt"][1], y], color=e["col"],
                    lw=0.7, alpha=0.6, zorder=2)
            ax.text(x_col, y, e["testo"], color=e["col"], fontsize=9.5, ha=ha, va="center", zorder=6)


def disegna_angoli(ax, T_list, theta, alpha, gruppi, presenti):
    asse = lambda i, k: T_list[i][:3, 0 if k == "x" else 2]
    chiave = lambda P: next(k for k in gruppi if np.linalg.norm(np.array(k) - P) < 1e-6)

    angoli = []                                     # (tipo, i, punto 3D, asse rot., asse partenza, asse arrivo)
    for i in range(1, 8):
        d_i = T_list[i - 1][:3, 2] @ (T_list[i][:3, 3] - T_list[i - 1][:3, 3])
        O_p = T_list[i - 1][:3, 3] + d_i * T_list[i - 1][:3, 2]     # O'_i
        angoli.append(("t", i, O_p, (i - 1, "z"), (i - 1, "x"), (i, "x")))
        angoli.append(("a", i, T_list[i][:3, 3], (i, "x"), (i - 1, "z"), (i, "z")))

    fantasmi = {}                                   # key -> [direzione, etichette]
    for tipo, i, P, k_rot, k_da, k_a in angoli:
        key = chiave(P)
        for (j, kk) in (k_rot, k_da, k_a):
            if j in gruppi[key]:
                continue
            v = asse(j, kk)
            nome = f"{kk}_{j}"
            esistente = next((a for a in presenti[key] if a["v"] @ v > 1 - 1e-6), None)
            if esistente is not None:
                if nome not in esistente["extra"]:
                    esistente["extra"].append(nome)
                continue
            lista = fantasmi.setdefault(key, [])
            g = next((g for g in lista if g[0] @ v > 1 - 1e-6), None)
            if g is None:
                lista.append([v, [nome]])
            elif nome not in g[1]:
                g[1].append(nome)

    # assi fantasma che coincidono con assi disegnati: etichetta grigia accanto a quella dell'asse
    for key, lista in presenti.items():
        for a in lista:
            if a["extra"]:
                destra = a["ha"] == "left"
                ax.annotate("$(" + r" \equiv ".join(a["extra"]) + ")$", xy=(1 if destra else 0, 0.5),
                            xycoords=a["artist"], xytext=(4 if destra else -4, 0),
                            textcoords="offset points", color=COL_GH, fontsize=9,
                            ha="left" if destra else "right", va="center")
    # assi fantasma tratteggiati
    for key, lista in fantasmi.items():
        p = pos_disegno(np.array(key))
        for v, nomi in lista:
            u = dir2d(v)
            ax.add_patch(FancyArrowPatch(p, p + 0.85 * L * u, arrowstyle="-|>", mutation_scale=12,
                                         color=COL_GH, lw=1.4, ls=(0, (3, 2)), zorder=2,
                                         shrinkA=0, shrinkB=0))
            q = p + 0.85 * L * u + (np.array([0.12, 0]) if abs(u[0]) < 0.3 else 0.14 * u)
            ha = "left" if (abs(u[0]) < 0.3 or u[0] > 0) else "right"
            ax.text(*q, "$(" + r" \equiv ".join(nomi) + ")$", color=COL_GH, fontsize=9,
                    ha=ha, va="center", bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.9), zorder=6)

    # archi e anelli
    etichette = []
    for tipo, i, P, k_rot, k_da, k_a in angoli:
        p = pos_disegno(P)
        phi = theta[i - 1] if tipo == "t" else alpha[i - 1]
        col = COL_TH if tipo == "t" else COL_AL
        nome = r"\theta" if tipo == "t" else r"\alpha"
        testo = f"${nome}_{i}={fmt_ang(phi)}$"
        if abs(phi) < 1e-9:                          # angolo nullo: anello sulla punta
            tip = p + L * dir2d(asse(*k_a))
            pt = anello(ax, tip, col)
            lato = -1
        else:
            pt = arco(ax, p, asse(*k_da), asse(*k_rot), phi, RAGGI[(tipo, i)], col)
            lato = 1 if pt[0] - p[0] > 0.02 else -1
        etichette.append(dict(pt=pt, lato=lato, testo=testo, col=col))
    piazza_etichette(ax, etichette)


def disegna(ax, schema):
    th = Q_REF + schema["offset"]
    T_list = terne(schema["tab"], th)

    # gruppi di terne con origine coincidente
    gruppi = {}
    for i, T in enumerate(T_list):
        key = tuple(np.round(T[:3, 3], 6))
        gruppi.setdefault(key, []).append(i)

    # braccio
    # pinza: base sopra O7, dita che si estendono sotto, O7 a meta' delle dita
    y_base, y_punta = Y_EE + 0.5, Y_EE - 0.5
    ax.plot([0, 0], [Y_SPALLA, y_base], color="0.6", lw=6, solid_capstyle="round", zorder=1)
    ax.plot([-0.9, 0.9], [y_base, y_base], color="0.6", lw=4, zorder=1)
    ax.plot([-0.9, -0.9], [y_base, y_punta], color="0.6", lw=4, zorder=1)
    ax.plot([0.9, 0.9], [y_base, y_punta], color="0.6", lw=4, zorder=1)

    con_anello = {f"x_{i}" for i in range(1, 8) if abs(th[i - 1]) < 1e-9}
    presenti = {}                                   # frecce disegnate per ogni origine
    for key, idx in gruppi.items():
        p = pos_disegno(np.array(key))
        ax.plot(*p, "o", color="k", ms=6, zorder=5)
        nomi = r" \equiv ".join(f"O_{i}" for i in idx)
        if idx == [7]:                                # O7 tra le dita, a destra del punto
            ax.text(p[0] + 0.14, p[1] + 0.16, f"${nomi}$", fontsize=10, ha="left", va="center", bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.9), zorder=6)
        else:
            ax.text(p[0] - 0.3, p[1] + 0.3, f"${nomi}$", fontsize=10, ha="right", va="center", bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.9), zorder=6)

        # assi x e z del gruppo, uniti quando coincidono
        assi = []                                   # [direzione (terna 0), etichette, tipi]
        for i in idx:
            for k, nome in ((0, "x"), (2, "z")):
                v = T_list[i][:3, k]
                for a in assi:
                    if np.dot(a[0], v) > 1 - 1e-6:
                        a[1].append(f"{nome}_{i}")
                        a[2].add(nome)
                        break
                else:
                    assi.append([v, [f"{nome}_{i}"], {nome}])

        presenti[key] = []
        for v, etichette, tipi in assi:
            col = COL[tipi.pop()] if len(tipi) == 1 else COL["mix"]
            u = proietta(v)
            u = u / np.linalg.norm(u)
            evidenzia = "x_4" in etichette
            freccia(ax, p, L * u, col, lw=3.2 if evidenzia else 2.0)
            testo = "$" + r" \equiv ".join(etichette) + "$"
            anello_qui = any(e in con_anello for e in etichette)
            if abs(u[0]) < 0.3:                     # freccia verticale: etichetta di lato
                q = p + L * u + np.array([0.3 if anello_qui else 0.12, 0.0])
                ha = "left"
            else:
                q = p + (L + (0.3 if anello_qui else 0.12)) * u
                ha = "left" if u[0] > 0 else "right"
            art = ax.text(*q, testo, color=col, fontsize=10, ha=ha, va="center",
                          fontweight="bold" if evidenzia else "normal",
                          bbox=dict(boxstyle="round,pad=0.15", fc="#fff3c4" if evidenzia else "white",
                                    ec="none", alpha=1 if evidenzia else 0.9), zorder=6)
            presenti[key].append(dict(v=v, q=q, ha=ha, extra=[], artist=art))

    disegna_angoli(ax, T_list, th, [r[2] for r in schema["tab"]], gruppi, presenti)

    # quote (con segno) a lato
    xq = 3.5
    quote = [(Y_SPALLA, Y_GOMITO, r"$d_3=-0.28$ m" + "\n" + r"(lungo $z_2$)"),
             (Y_GOMITO, Y_POLSO, (r"$a_4=+0.28$ m" if schema["tab"][3][0] > 0 else r"$a_4=-0.28$ m")
              + "\n" + r"(lungo $x_4$)"),
             (Y_POLSO, Y_EE, r"$d_7=-0.11$ m" + "\n" + r"(lungo $z_6$)")]
    for y1, y2, testo in quote:
        ax.annotate("", (xq, y2), (xq, y1),
                    arrowprops=dict(arrowstyle="<->", color="0.3", lw=1.3))
        ax.text(xq + 0.12, (y1 + y2) / 2, testo, color="0.2", fontsize=9.5, va="center")
        for y in (y1, y2):
            ax.plot([3.0, xq + 0.08], [y, y], color="0.5", lw=0.6, ls=":")

    ax.set_title(schema["titolo"] + "\n" + schema["sottotitolo"], fontsize=11, pad=10)
    ax.set_xlim(-3.6, 5.2)
    ax.set_ylim(-8.0, 1.5)
    ax.set_aspect("equal")
    ax.axis("off")


def legenda_vista(fig):
    ax = fig.add_axes([0.455, 0.06, 0.09, 0.11])
    o = np.zeros(2)
    for v, nome, c in ((P_X0, "x_0", COL["x"]), (P_Z0, "z_0", COL["z"]), (P_Y0, "y_0", "#2ca02c")):
        u = v / np.linalg.norm(v)
        freccia(ax, o, 0.7 * u, c, lw=1.4)
        ax.text(*(0.95 * u), f"${nome}$", color=c, fontsize=9, ha="center", va="center")
    ax.text(0, -1.2, "vista: terna 0\n($y_0$ uscente)", fontsize=8, ha="center", va="top")
    ax.set_xlim(-1.1, 1.1); ax.set_ylim(-1.6, 1.1); ax.set_aspect("equal"); ax.axis("off")


def main():
    fig, axs = plt.subplots(1, 2, figsize=(16, 11.5))
    for ax, schema in zip(axs, SCHEMI):
        disegna(ax, schema)
    legenda_vista(fig)
    fig.suptitle(r"Terne DH del braccio sinistro nella posa di riferimento "
                 r"$q=[0,\,-\pi/2,\,-\pi/2,\,-\pi/2,\,0,\,\pi/2,\,0]$" "\n"
                 r"assi $x$ in rosso, assi $z$ in blu, assi $x \equiv z$ coincidenti in viola; "
                 r"distanze non in scala",
                 fontsize=12)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.86, bottom=0.02, wspace=0.05)
    fig.savefig("schema_terne_dh.png", dpi=200)
    fig.savefig("schema_terne_dh.pdf")
    print("Salvati schema_terne_dh.png e schema_terne_dh.pdf")


if __name__ == "__main__":
    main()
