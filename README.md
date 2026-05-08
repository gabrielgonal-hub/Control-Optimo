![Python](https://img.shields.io/badge/Python-3.x-blue)
![ESP32](https://img.shields.io/badge/ESP32-Control-green)
![LaTeX](https://img.shields.io/badge/LaTeX-Document-red)
![License](https://img.shields.io/badge/License-MIT-yellow)

# Control Óptimo de un Sistema RC con ESP32

Proyecto desarrollado para la unidad de aprendizaje de **Control Óptimo** en la Facultad de Ingeniería Mecánica y Eléctrica (FIME-UANL).

---

# Modelo del sistema

Las variables de estado se definieron como:

$$
x_1 = V_1
$$

$$
x_2 = V_2
$$

con salida:

$$
y = x_2
$$

La representación en espacio de estados está dada por:

$$
\dot{x} = Ax + Bu
$$

$$
y = Cx
$$

donde:

$$
A =
\begin{bmatrix}
-\frac{1}{R_1 C_1} - \frac{1}{R_2 C_1} & \frac{1}{R_2 C_1} \\
\frac{1}{R_2 C_2} & -\frac{1}{R_2 C_2} - \frac{1}{R_3 C_2}
\end{bmatrix}
$$

$$
B =
\begin{bmatrix}
\frac{1}{R_1 C_1} \\
0
\end{bmatrix}
$$

$$
C =
\begin{bmatrix}
0 & 1
\end{bmatrix}
$$

---

# Control óptimo LQR

El controlador óptimo fue calculado resolviendo la ecuación algebraica de Riccati:

```python
P = solve_continuous_are(A,B,Q,R)
K = np.linalg.inv(R) @ B.T @ P
```

Resultado:

$$
K =
\begin{bmatrix}
0.41421356 & 0.41421356
\end{bmatrix}
$$

---

# Control óptimo con acción integral (LQI)

El sistema fue aumentado incorporando acción integral.

$$
\dot{X}_a = A_aX_a + B_au
$$

con:

$$
A_a =
\begin{bmatrix}
0 & -C \\
0 & A
\end{bmatrix}
$$

$$
A_a =
\begin{bmatrix}
0 & 0 & -1 \\
0 & a_{11} & a_{12} \\
0 & a_{21} & a_{22}
\end{bmatrix}
$$

y función de costo:

$$
Q_a =
\begin{bmatrix}
10 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 5
\end{bmatrix}
$$

---

# Observador de Luenberger

$$
\dot{\hat{x}} =
A\hat{x} + Bu + H(y-C\hat{x})
$$

---

# Seguimiento de referencia

$$
u = -Kx + gr
$$

donde:

$$
g =
\begin{bmatrix}
K & 1
\end{bmatrix}
\begin{bmatrix}
A & B \\
C & D
\end{bmatrix}^{-1}
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
$$

---

# Índice de desempeño

El controlador LQR fue diseñado mediante la minimización del funcional cuadrático:

$$
J = \int_{0}^{\infty}
\left(
x^TQx + u^TRu
\right)dt
$$

Utilizando:

$$
Q =
\begin{bmatrix}
1 & 0 \\
0 & 5
\end{bmatrix}
$$

$$
R = 1
$$

Este criterio permite balancear:

- Seguimiento de referencia
- Energía de control
- Rapidez de respuesta

---

# Controladores implementados

## 1. Asignación de polos

Se diseñó una ley de control por retroalimentación de estados:

$$
u = -Kx + gr
$$

Las ganancias se obtuvieron igualando el polinomio característico del sistema en lazo cerrado con un polinomio deseado.

---

## 2. Control integral por asignación de polos

Se incorporó un integrador:

$$
x_0 = \int (r-y)dt
$$

permitiendo eliminar el error en estado estacionario para referencias constantes.

---

## 3. Control óptimo LQR

El controlador óptimo fue calculado resolviendo la ecuación algebraica de Riccati:

```python
P = solve_continuous_are(A,B,Q,R)
K = np.linalg.inv(R) @ B.T @ P
````

Resultado:

$$
K =
\begin{bmatrix}
0.41421356 & 0.41421356
\end{bmatrix}
$$

---

## 4. Control óptimo con acción integral (LQI)

El sistema fue aumentado incorporando acción integral y posteriormente se resolvió nuevamente la ecuación de Riccati.

El sistema aumentado utilizado fue:

$$
\dot{X}_a = A_aX_a + B_au
$$

con:

$$
A_a =
\begin{bmatrix}
0 & 0 & -1 \
0 & & \
0 & & A
\end{bmatrix}
$$

y función de costo:

$$
Q_a = diag(10,1,5)
$$

El controlador LQI permitió eliminar el error estacionario manteniendo un enfoque óptimo sobre el sistema aumentado.

---

# Observador de Luenberger

Para los controladores basados en asignación de polos se implementó un observador de estados debido a que experimentalmente sólo se mide el segundo capacitor.

El observador utilizado fue:

$$
\dot{\hat{x}} =
A\hat{x} + Bu + H(y-C\hat{x})
$$

Los polos del observador se seleccionaron considerablemente más rápidos que los polos del controlador para garantizar convergencia rápida del error de estimación.

---

# Seguimiento de referencia

Para garantizar seguimiento sin error estacionario se implementó una ganancia de prealimentación:

$$
u = -Kx + gr
$$

donde:

$$
g =
\begin{bmatrix}
K & 1
\end{bmatrix}
\begin{bmatrix}
A & B \
C & D
\end{bmatrix}^{-1}
\begin{bmatrix}
0 \
0 \
1
\end{bmatrix}
$$

---

# Implementación experimental

La validación experimental se realizó utilizando:

* ESP32
* 4 plantas RC físicamente equivalentes
* Adquisición analógica en tiempo real
* Señales PWM
* Comunicación serial

Cada planta fue asociada a un controlador distinto para permitir comparaciones simultáneas bajo las mismas condiciones experimentales.

---

# Sistema de monitoreo

Se desarrolló una interfaz en Python utilizando:

* PySide6
* PyQtGraph
* pandas

Características:

* Visualización en tiempo real
* Monitoreo simultáneo de los 4 controladores
* Exportación automática a CSV
* Evaluación automática del índice de desempeño

---

# Evaluación experimental

El índice de desempeño se calculó numéricamente mediante:

```python
J = ((x1**2 + 5*x2**2 + u**2) * dt).sum()
```

Los resultados experimentales mostraron que:

* El controlador LQR obtuvo el menor valor de (J)
* LQI presentó mejor seguimiento pero mayor costo total
* Los métodos por asignación de polos tuvieron mayor esfuerzo de control

---

# Resultados principales

✔ Modelado matemático completo del sistema RC
✔ Diseño de múltiples estrategias de control moderno
✔ Implementación física en ESP32
✔ Sistema de monitoreo en tiempo real
✔ Validación experimental del controlador LQR
✔ Comparación experimental entre estrategias óptimas y no óptimas

---

# Tecnologías utilizadas

## Hardware

* ESP32
* Protoboard
* Resistencias
* Capacitores

## Software

* Python
* NumPy
* SciPy
* Matplotlib
* PySide6
* PyQtGraph
* pandas
* Arduino IDE
* LaTeX

---

# Referencias

1. K. Ogata, *Modern Control Engineering*, 5th ed.
2. D. E. Kirk, *Optimal Control Theory: An Introduction*
3. G. F. Franklin, *Feedback Control of Dynamic Systems*

---

# Autor

**Gabriel González Alvarez**
Facultad de Ingeniería Mecánica y Eléctrica
Universidad Autónoma de Nuevo León

---
