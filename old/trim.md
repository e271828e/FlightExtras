Update (last)

Nota previa: todas las variables de actitud, en este contexto, se refieren a la rotacion $nb$. Pero las velocidades angulares son $lb$.

Total variables a determinar (solo las relevantes en vuelo):
Derivatives: $\dot{\psi}, \dot{\theta}, \dot{\phi}, \dot{\varphi}, \dot{\lambda}, \dot{h}, \dot{p}, \dot{q}, \dot{r}, \dot{v}_x, \dot{v}_y, \dot{v}_z, \dot{\omega}_{eng}, \dot{x}_{idle}, \dot{x}_{fr}, \dot{x}_{fuel}, \dot{\alpha}_{filt}, \dot{\beta}_{filt}$ (18)
States: $\psi, \theta, \phi, \varphi, \lambda, h, p, q, r, v_x, v_y, v_z, \omega_{eng}, x_{idle}, x_{fr}, x_{fuel}, \alpha_{filt}, \beta_{filt}$ (18)
Inputs: throttle, elevator, aileron, rudder, flaps, mixture (6)
Total: 42

System dynamics:
Kinematic equations: $\dot{\psi}, \dot{\theta}, \dot{\phi}, \dot{\varphi}, \dot{\lambda}, \dot{h}$
Rigid body equations: $\dot{p}, \dot{q}, \dot{r}, \dot{v}_x, \dot{v}_y, \dot{v}_z$
Airframe subsystems equations: $\dot{\omega}_{eng}, \dot{x}_{fuel}, \dot{x}_{idle}, \dot{x}_{fr}, \dot{\alpha}_{filt}, \dot{\beta}_{filt}$ (6)
Total: 18

Las state derivatives las determinan directamente con las ecuaciones dinamicas del sistema, por lo que puedo eliminar ambos grupos. Quedan entonces states + inputs. (24)

Tras la asignacion directa de $\psi, \varphi, \lambda, h, x_{idle}, x_{fr}, x_{fuel}$, flaps, mixture (9),  quedan los siguientes grados de libertad:

$\theta, \phi, p, q, r, v_x, v_y, v_z, \omega_{eng}, \alpha_{filt}, \beta_{filt}$, throttle, elev, ail, rud (15)

Ahora, por comodidad es perfectamente valido sustituir las componentes de la velocidad aerodinamica en ejes cuerpo por $TAS, \alpha, \beta$, con lo que quedaria:

$\theta, \phi, p, q, r, TAS, \alpha, \beta, \omega_{eng}, \alpha_{filt}, \beta_{filt}$, throttle, elev, ail, rud (15)

Ahora, para asegurar $\dot{\alpha}_{filt} = \dot{\beta}_{filt}=0$, es necesario asignar $\alpha_{filt}=\alpha, \beta_{filt} = \beta$. Con esto quedan:

$\theta, \phi, p, q, r, TAS, \alpha, \beta, \omega_{eng}$, throttle, elev, ail, rud (13)

Por tanto, hay que imponer 13 constraints.

Las dos primeras consisten en definir directamente los valores de $TAS, \beta$. Otras tres constraints a imponer son:  $\dot{\theta} = \dot{\phi} = 0$ y un valor arbitrario del turn rate $\dot{\psi}$. Fijados los grados de libertad $\psi, \theta, \phi$, estas tres constraints nos proporcionan directamente $p, q, r$ a través de la relación cinemática:

$\omega_{lb}^b = \omega_{lb}^b(\dot{\psi}, \dot{h}, \dot{\theta}, \dot{\phi},\psi, \theta, \phi, )$

Al ser $\dot{\psi}, TAS$ y $\beta$ de imposicion directa, podemos agruparlas con los parametros:

$TAS, \beta, \dot{\psi}, \psi, \varphi, \lambda, h, x_{idle}, x_{fr}, x_{fuel}$, flaps, mixture

Por tanto, quedan como grados de libertad:

$\theta, \phi, \alpha, \omega_{eng}$, throttle, elev, ail, rud (8)

%%%%%%%%%%%%

Sean $x^\alpha$ las componentes de un vector $x$ en ejes $\alpha$. Consideremos dos rotaciones secuenciales partiendo de ejes $a$: una primera $R_z(\chi)$ y una segunda $R_y(\theta)$, para obtener unos ejes $\beta$ en los que:
$$
x^\beta = \|x\|[1,0,0]^T
$$

Llamamos a $\chi$ y $\theta$ respectivamente azimuth e inclinacion.

Esta rotacion compuesta se puede expresar como:
$$x^\alpha = R_z(\chi) R_y(\theta) x^\beta =
\|x\|
\begin{pmatrix} \cos\chi & -\sin\chi & 0 \\
                                \sin\chi & \cos\chi & 0 \\
                                0 & 0 & 1 \end{pmatrix}
\begin{pmatrix} \cos\gamma & 0 & \sin\gamma \\
                                0        & 1 &          0 \\
                                -\sin\gamma & 0 & \cos\gamma \end{pmatrix}
\begin{pmatrix} 1\\0\\0\end{pmatrix} = \\
\|x\| \begin{pmatrix} \cos\chi & -\sin\chi & 0 \\
                                \sin\chi & \cos\chi & 0 \\
                                0 & 0 & 1 \end{pmatrix}
\begin{pmatrix} \cos \gamma\\0\\ -\sin \gamma\end{pmatrix} =
\|x\| \begin{pmatrix} \cos \gamma \cos \chi \\ \cos \gamma \sin \chi \\ -\sin \gamma\end{pmatrix} \\
$$

Por tanto:
$$
\chi = \arctan \dfrac{x^{\alpha[2]}}{x^{\alpha[1]}}\\
\gamma = \arctan \dfrac{-x^{\alpha[3]}}{\sqrt{(x^{\alpha[1]})^2+(x^{\alpha[2]})^2}}\\
$$

Cuando $x$ es una velocidad $v_{rOb}^n$, donde $\alpha=n$ son los ejes NED y $r$ es un reference frame arbitrario (que generalmente sera $w$ o $e$), entonces $\chi = \chi_{rOb}^n$ y $\gamma = \gamma_{rOb}^n$ se conocen respectivamente como track angle y flight path angle relativos a $r$.

Para $r = w$ tenemos:
$$v_{wOb}^n =TAS \begin{pmatrix} \cos \gamma_{wOb}^n \cos \chi_{wOb}^n \\ \cos \gamma_{wOb}^n \sin \chi_{wOb}^n \\ -\sin \gamma_{wOb}^n \end{pmatrix} \\
$$

Por otra parte:
$$v_{wOb}^n = R^n_b(\psi, \theta, \phi) v_{wOb}^b$$

Donde $v_{wOb}^b = R^b_a v_{wOa}^a (TAS, \alpha_a, \beta_a)$.

Igualando la tercera componente de ambas relaciones resulta:
$$-TAS \sin \gamma_{wOb}^n = -v_{wOb}^{b[1]} \sin \theta + v_{wOb}^{b[2]} \cos \theta \cos \phi + v_{wOb}^{b[3]} \cos \theta \sin \phi = -v_{wOb}^{b[1]} \sin \theta + (v_{wOb}^{b[2]} \cos \phi + v_{wOb}^{b[3]} \sin \phi) \cos \theta$$

$$\sin \gamma_{wOb}^n = a \sin \theta - b \cos \theta$$

Donde:
$$
a = \dfrac{v_{wOb}^{b[1]}}{TAS}  \\
b =\dfrac{v_{wOb}^{b[2]}}{TAS} \sin \phi + \dfrac{v_{wOb}^{b[3]}}{TAS} \cos \phi
$$

Ahora se trata de obtener $\theta(\gamma, a, b)$:
$$
\sin^2 \gamma = a^2 \sin^2 \theta + b^2 \cos^2 \theta - 2ab \sin \theta \cos \theta\\
$$

Dividiendo por $\cos^2 \theta$:
$$
\sin^2 \gamma \sec^2 \theta = a^2 \tan^2 \theta + b^2 - 2ab \tan \theta\\
\sin^2 \gamma + \sin^2 \gamma \tan^2 \theta = a^2 \tan^2 \theta + b^2 - 2ab \tan \theta\\
(a^2 - \sin^2 \gamma) \tan^2 \theta - 2ab \tan \theta + b^2 - \sin^2 \gamma = 0\\
$$

Esto es una ecuacion cuadratica en $\tan \theta$. Resolvemos:
$$
\tan \theta = \dfrac{ab \pm \sin \gamma \sqrt{a^2 + b^2 - \sin^2 \gamma}}{a^2- \sin^2\gamma}
$$

Ahora hay que determinar el signo correcto.

Consideremos un caso en que el aerodynamic frame $\varepsilon_a$ (en el que se miden $\alpha$ y $\beta$) y el vehicle frame $\varepsilon_b$ son coincidentes, tenemos:
$$
v_{wOb}^{b[1]} = v_{wOa}^{a[1]} = TAS \cos \alpha \cos \beta\\
v_{wOb}^{b[2]} = v_{wOa}^{a[2]} = TAS \sin \beta\\
v_{wOb}^{b[3]} = v_{wOa}^{a[3]} = TAS \sin \alpha \cos \beta\\
$$

Con lo que:
$$
a = \cos \alpha \cos \beta \\
b = \sin \beta \sin \phi + \sin \alpha \cos \beta \cos \phi
$$

Si ahora consideramos el caso particular $\phi = \beta = 0$:
$$
a = \cos \alpha\\
b = \sin \alpha
$$

Introduciendo esto en la solucion:
$$
\tan \theta = \dfrac{\cos \alpha \sin \alpha \pm \sin \gamma \sqrt{1 - \sin^2 \gamma}}{\cos^2 \alpha- \sin^2\gamma}
= \dfrac{\cos \alpha \sin \alpha \pm \sin \gamma \cos \gamma}{\cos^2 \alpha- \sin^2\gamma}
$$

Introduciendo valores de $\alpha$ y $\gamma$, se comprueba que para obtener $\theta = \gamma + \alpha$ el signo correcto es $+$.Deberia ser posible comprobar esta identidad mediante manipulacion e identidades trigonometricas, pero paso.

El resultado de todo esto es una ligadura que, habiendo obtenido previamente $TAS, \alpha, \beta$ (que nos proporcionan $v_{wOb}^b$) y $\phi$, e imponiendo $\gamma_{wOb}^n$, podemos despejar $\theta$. Igual que hicimos con $TAS$, $\beta$ y $\dot{\psi}$, incorporamos $\gamma_w^n$ a los trim parameters.

Tras introducir esta ligadura, nos quedan 7 grados de libertad con 7 parametros libres. Y esos los cerramos imponiendo las siguientes 7 condiciones escalares a traves de la funcion de coste:
$$
\dot{\omega}_{lb}^b = 0 \\
\dot{v}_{eOb}^b = 0 \\
\dot{\omega}_{eng} = 0
$$

Quedan entonces:
- Trim parameters: $TAS, \gamma_{w}^n, \beta, \dot{\psi}, \psi, \varphi, \lambda, h, x_{idle}, x_{fr}, x_{fuel}$, flaps, mixture
- Trim state: $\alpha_a, \phi_{nb}, \omega_{eng}$, throttle, ele, ail, rud (7)
- Trim cost function constraints: $\dot{\omega}_{lb}^b = \dot{v}_{eOb}^b = \dot{\omega}_{eng} = 0$ (7)

Con esto queda cerrado el problema.