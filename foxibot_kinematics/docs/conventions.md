# Foxibot – Leg Kinematics Conventions

## 1. Conventions géométriques

### 1.1 Repère de référence et origine

Le repère de référence de la patte est noté $\mathcal{R}_H$ et est attaché à la hanche.

* **Origine :** centre de rotation de l’articulation $J_1$.
* **Convention utilisée :** convention ROS REP-103.
* Toutes les positions du pied sont exprimées dans le repère $\mathcal{R}_H$.
* La position du pied est notée :

$$
\mathbf{p} =
\begin{pmatrix}
x \
y \
z
\end{pmatrix}
$$

où $x$, $y$ et $z$ sont exprimés en mètres.

### 1.2 Orientation des axes

Les axes du repère $\mathcal{R}_H$ sont orientés comme suit :

* **Axe $X$ :** vers l’avant du robot.
* **Axe $Y$ :** vers la gauche du robot.
* **Axe $Z$ :** vers le haut du robot.

```text
         Z
         ^
         |
         |
         o --------> X
        /
       /
      Y
```

Cette orientation est commune aux quatre pattes. Les éventuelles différences entre les pattes gauche et droite devront être traitées par leurs transformations respectives par rapport au corps du robot, et non en modifiant la convention locale de la patte.

### 1.3 Architecture de la patte

La patte possède trois articulations rotatives :

| Articulation | Désignation                      | Axe de rotation | Angle associé |
| ------------ | -------------------------------- | --------------: | ------------: |
| $J_1$        | Abduction/adduction de la hanche |             $X$ |    $\theta_1$ |
| $J_2$        | Flexion/extension de la hanche   |             $Y$ |    $\theta_2$ |
| $J_3$        | Flexion/extension du genou       |             $Y$ |    $\theta_3$ |

L’articulation $J_1$ permet de faire tourner le plan de la jambe autour de l’axe $X$.

Les articulations $J_2$ et $J_3$ forment ensuite une chaîne cinématique plane évoluant initialement dans le plan $XZ$ lorsque :

$$
\theta_1 = 0
$$

**NB**: cette condition n'est vraie que dans le plan $XZ$, sinon $\theta_1$ varie entre $(-\pi,\pi]$ dans les limites mécaniques de mobilité.

### 1.4 Dimensions géométriques

Les longueurs utilisées dans le modèle cinématique sont :

* $L_0$ : distance entre le centre de rotation de $J_1$ et le centre de rotation de $J_2$ ;
* $L_1$ : longueur du fémur, entre $J_2$ et $J_3$ ;
* $L_2$ : longueur du tibia, entre $J_3$ et le pied.

| Paramètre | Description                 |       Valeur |
| --------- | --------------------------- | -----------: |
| $L_0$     | Offset entre $J_1$ et $J_2$ | À déterminer |
| $L_1$     | Longueur du fémur           | À déterminer |
| $L_2$     | Longueur du tibia           | À déterminer |

Toutes les longueurs sont exprimées en mètres.

> [!NOTE]
> Dans le code, $L_0$ correspond au champ `hip_offset`, $L_1$ à `femur_length` et $L_2$ à `tibia_length`.

### 1.5 Position zéro

La position articulaire zéro est définie par :

$$
\theta_1 = 0,\qquad
\theta_2 = 0,\qquad
\theta_3 = 0
$$

Dans cette configuration :

* le plan de la jambe est confondu avec le plan $XZ$ ;
* le fémur et le tibia sont alignés ;
* la patte est entièrement tendue vers le bas ;
* le pied est situé sous la hanche.

La position exacte du pied dépend de la convention retenue pour l’offset $L_0$.

Elle devra être écrite explicitement après validation du schéma cinématique :

$$
\mathbf{p}_0 =
\begin{pmatrix}
0 \
L_0 \
-(L_1+L_2)
\end{pmatrix}
$$

### 1.6 Sens positifs des articulations

Le sens positif de chaque articulation est défini à l’aide de la règle de la main droite autour de son axe de rotation.

#### Articulation $J_1$

* Axe de rotation : $X$.
* Angle associé : $\theta_1$.
* Sens positif : règle de la main droite autour de $+X$.
* Conséquence physique : 
    * $\theta_1>0$ : la patte tourne vers le sol
    * $\theta_1<0$ : la patte tourne vers le ciel

#### Articulation $J_2$

* Axe de rotation : $Y$.
* Angle associé : $\theta_2$.
* Sens positif : règle de la main droite autour de $+Y$.
* Convention physique retenue : un angle positif déplace le fémur vers l’avant du robot, dans la direction $+X$.

#### Articulation $J_3$

* Axe de rotation : $Y$ local de l’articulation du genou.
* Angle associé : $\theta_3$.
* Sens positif : règle de la main droite autour de cet axe.
* Convention physique retenue : à préciser selon le sens de flexion choisi pour obtenir la configuration « genou arrière ».

> [!WARNING]
> Les sens positifs de $J_1$ et $J_3$ doivent être validés graphiquement sur les schémas avant l’implémentation. Une erreur de signe dans cette section se propagerait dans la FK, l’IK, l’URDF et la calibration des servomoteurs.

### 1.7 Poses de référence

Trois poses sont définies afin de vérifier les conventions et les équations cinématiques.

Ces poses seront utilisées comme cas de test pour :

* la cinématique directe ;
* la cinématique inverse ;
* les tests aller-retour FK → IK → FK.

#### Pose de référence A — Position zéro

Angles :

$$
\theta_1 = 0,\qquad
\theta_2 = 0,\qquad
\theta_3 = 0
$$

Position attendue :

$$
\mathbf{p}_A =
\begin{pmatrix}
0 \
L_0 \
-(L_1+L_2)
\end{pmatrix}
$$

#### Pose de référence B — Flexion de hanche à 90°

Angles :

$$
\theta_1 = 0,\qquad
\theta_2 = \frac{\pi}{4},\qquad
\theta_3 = \frac{\pi}{2},\qquad
$$

Position attendue :

$$
\mathbf{p}_B =
\begin{pmatrix}
\frac{L_1+L2}{\sqrt{2}}\
L_0 \
\frac{L_1-L2}{\sqrt{2}}
\end{pmatrix}
$$

#### Pose de référence C — Pose quelconque

Angles :

$$
\theta_1 = \frac{\pi}{4},\qquad
\theta_2 = 0,\qquad
\theta_3 = \frac{\pi}{2},\qquad
$$

Position attendue :

$$
\mathbf{p}_C =
\begin{pmatrix}
0\
\frac{L_0+L_1}{\sqrt{2}} \
\frac{L_0-L1}{\sqrt{2}}
\end{pmatrix}
$$

La troisième pose doit comporter des angles non nuls sur les trois articulations afin de vérifier le comportement complet de la cinématique 3D.

### 1.8 Limites articulaires

Les limites articulaires sont définies dans le modèle de la patte :

| Articulation | Angle minimal | Angle maximal |
| ------------ | ------------: | ------------: |
| $J_1$        |  À déterminer |  À déterminer |
| $J_2$        |  À déterminer |  À déterminer |
| $J_3$        |  À déterminer |  À déterminer |

Les angles sont exprimés en radians dans le code.

La cinématique directe calcule une position sans modifier ni limiter les angles fournis.

La cinématique inverse calcule une solution géométrique. La validité de cette solution vis-à-vis des limites articulaires est vérifiée séparément par la fonction `isWithinLimits()`.

### 1.9 Convention de résolution de la cinématique inverse

Pour une même position du pied, plusieurs configurations articulaires peuvent être possibles.

Dans la première version de Foxibot, une seule branche de solution est retenue :

* configuration dite **genou vers l’arrière** ;
* l’autre solution géométrique n’est pas retournée ;
* le choix du signe utilisé dans le calcul de $\theta_3$ devra être indiqué dans la partie consacrée à l’IK.

La fonction de cinématique inverse retourne un `std::optional<JointAngles>`.

Elle retourne `std::nullopt` lorsque la cible est géométriquement inatteignable, notamment lorsque la distance entre $J_2$ et le pied est :

$$
r > L_1 + L_2
$$

ou :

$$
r < \left|L_1 - L_2\right|
$$

Une cible est également géométriquement inatteignable lorsque sa distance à l’axe de rotation de la hanche est inférieure à l’offset $L_0$.

Dans le plan $(YZ)$, cette distance vaut :

$$
\rho = \sqrt{y^2+z^2}
$$

La condition d’existence d’une solution pour $\theta_1$ est donc :

$$
\rho \ge |L_0|
$$

soit, de manière équivalente :

$$
y^2+z^2 \ge L_0^2
$$

Si cette condition n’est pas respectée, aucun plan de jambe obtenu par rotation autour de $J_1$ ne peut contenir la cible. La cinématique inverse retourne alors `std::nullopt`.

Les frontières exactes sont considérées comme atteignables :

$$
r = L_1 + L_2
$$

correspond à une jambe entièrement tendue, tandis que :

$$
r = \left|L_1 - L_2\right|
$$

correspond à une jambe entièrement repliée.



> [!NOTE]
> Une cible atteignable géométriquement mais produisant des angles hors des limites articulaires ne provoque pas nécessairement un `std::nullopt`. La portée géométrique et les limites mécaniques constituent deux vérifications distinctes.


## 2. Schémas cinématiques

Les trois représentations suivantes décrivent le modèle cinématique utilisé pour établir les équations de la cinématique directe et inverse.

| Vue 2D (XZ) | Vue 2D (YZ) | Vue 3D |
|:-----------:|:-----------:|:------:|
| Schéma de la cinématique plane utilisée pour les calculs de $\theta_2$ et $\theta_3$. | Projection utilisée pour le calcul de $\theta_1$ et la prise en compte de l'offset $L_0$. | Représentation complète de la patte dans son repère de référence. |

<p align="center">
  <img src="images/kinematics_schematic.jpeg" alt="Schémas cinématiques de la patte Foxibot" width="2700"/>
</p>

**Figure 1 —** Représentations cinématiques de la patte Foxibot : vue dans le plan $XZ$, vue dans le plan $YZ$ et représentation 3D.

## 3. Forward Kinematics (FK)

La cinématique directe détermine la position du pied $\mathbf p=(x,y,z)^T$ à partir des angles $(\theta_1,\theta_2,\theta_3)$.

### 3.1 Calcul dans le plan de la jambe

<p align="center">
  <img src="images/plan_xz_sagittal_theta2_theta3.png" alt="Schémas cinématiques de la patte Foxibot - Plan $(X Z)" width="2700"/>
</p>

Dans le plan de la jambe $(X Z)$, le résulat des  calcules obtenu à partir des données définies dans les parties précédentes donne :

$
x_1 = L_1.sin(\theta_2)+L_2.sin(\alpha)
$

$
z_1 = -(L_1.cos(\theta_2)+L_2.cos(\alpha))
$

avec $\alpha = \theta_2+\theta_3$.

On pose $A=L_1.cos(\theta_2)+L_2.cos(\alpha)$ tel que $z_1 = -A$

> [Warning]
> Le signe de $z_1$ est obtenu car la jambe est orienté vers le bas par convention.


### 3.2 Rotation autour de J1

En changement la base des calculs précédent de la base de J2 à J1, on a : 

$\vec{x_1}=\vec{x_1}$

$\vec{y_1}=cos(\theta_1).\vec{y_0}+sin(\theta_1).\vec{z_0}$

$\vec{z_1}=cos(\theta_1).\vec{z_0}-sin(\theta_1).\vec{y_0}$

d'où, en passant de la base 1 à 0, on a: 

$L_0.\vec{y_1}=L0.cos(\theta_1).\vec{y_0}+L_0.sin(\theta_1).\vec{z_0}$

$x_1.\vec{x_1}=x_1.\vec{x_0} $

$z_1.\vec{z_1}=-A.\vec{z_0}-(-(A)).sin(\theta_1).\vec{y_0}$

### 3.3 Résultat final

Le vecteur $p(\theta_1,\theta_2,\theta_3)$ est : 

$p=\begin{pmatrix}
L_1.sin(\theta_2)+L_2.sin(\alpha) \\
L_0.cos(\theta_1)+(L_1.cos(\theta_2)+L_2.cos(\alpha)).sin(\theta_1) \\
L_0.sin(\theta_1)-(L_1.cos(\theta_2)+L_2.cos(\alpha)).cos(\theta_1)
\end{pmatrix}$

avec $\alpha=\theta_2+\theta_3$.

On peut aussi écrire cette matrice sous la forme d'un produit matricielle entre une matrice de rotation et un vecteur colonne. Cela donnerait :

$$
{}^{0}\vec{p}
={}^{0}R_{1}{}^{1}p=
\begin{pmatrix}
1 & 0 & 0 \\
0 & \cos\theta_1 & -\sin\theta_1 \\
0 & \sin\theta_1 & \cos\theta_1
\end{pmatrix}
\begin{pmatrix}
x_1 \\
L_0 \\
z_1
\end{pmatrix}_{\mathcal{R}_1}$$

## 4. Inverse Kinematics (IK)

La cinématique inverse détermine les angles articulaires à partir
d’une position cible \(\mathbf p=(x,y,z)^T\).

Les angles sont calculés dans l’ordre suivant :

1. calcul de θ1 ;
2. réduction au problème plan ;
3. calcul de θ3 ;
4. calcul de θ2.

### 4.1 Calcul de θ1

On se place dans le plan de la jambe. On projette le vecteur $\vec{p}$ dans la base (y,z).

$$
P_{(YZ)}=
\begin{cases}
y = L_0.cos(\theta_1)+ A.sin(\theta_1) \\
z= L0.sin(th1)-\left(L_1\cos(\theta_2) + L_2\cos(\alpha)\right)
\end{cases}
$$

On cherche à résoulde ce système à deux équation et deux inconnus, afin de trouver les expressions algébriques de $cos(\theta_1)$ et $sin(\theta_1)$ en fonction de $L_0$ et $A=L_1.cos(\theta_2)+L_2.cos(\alpha)$ avec $\alpha = \theta_1 + \theta_2$.

On trouve : 

$$
P_{(X Z)}=
\begin{cases}
cos(\theta_1)=\frac{y.L_0-A.z}{{L_0}^{2}+{A}^{2}} \\
sin(th1)=\frac{L_0.z+A.y}{{L_0}^{2}+{A}^{2}}
\end{cases}
$$

ainsi, on aura :

$$
\theta_1=atan²(\frac{L_0.z+A.y}{y.L_0-A.z})
$$

> [Note]
> La fonction `atan2(y, x)` c++ utilisée à la place de `atan` dans `leg_kinematics.cpp` car elle retourne des angles dans 4 quadrants du cercle trigonométique.
>
> Elle sera implémenté comme suit : `atan2(L_0.z+A.y,y.L_0-A.z)`
>
> `x` et `y` correspondent respectivement à cosinus et sinus. Leur sens est inversé pour correspondre à la fonction mathématique $arctan(tan(x))=arctan(\frac{sin(x)}{cos(x)})$.
### 4.2 Calcul r pour trouver $\theta_3$

On se place dans le plan de la jambe. On projette le vecteur $\vec{p}$ dans la base (X,Z).

$$p=
\begin{cases}
x = L_1\sin(\theta_2) + L_2\sin(\alpha) \\
z = L_0.sin(\theta_1)-\left(L_1\cos(\theta_2) + L_2\cos(\alpha)\right).cos(\theta_1)
\end{cases}
$$

Or, dans le plan $(X, Z)$ l'angle $\theta_1$ s'annule. On a alors:

$$p=
\begin{cases}
x = L_1\sin(\theta_2) + L_2\sin(\alpha) \\
z = -\left(L_1\cos(\theta_2) + L_2\cos(\alpha)\right).cos(\theta_1)
\end{cases}
$$

soit

$$p=
\begin{cases}
x = L_1\sin(\theta_2) + L_2\sin(\alpha) \\
-A = -\left(L_1\cos(\theta_2) + L_2\cos(\alpha)\right).cos(\theta_1)
\end{cases}
$$

On cherche r la distance entre J0 et le pied de la jambe, en utilisant la formule de distance euclidiènne. On a :

$$
r=\sqrt{x² + A²}
$$

On obtient : 

$$
r² = L_1²+L_2²+2.L_1.L_2.cos(\theta_3)
$$

La loi des cosinus appliquée au triangle J2–J3–pied donne :
$$cos⁡(θ3)= \frac{r^2 - L_1^2 - L_2^2}{2 \cdot L_1 \cdot L_2}​$$
Cette équation ne détermine pas θ₃ de manière unique. Puisque la fonction cosinus est paire (cos θ = cos(−θ)), les deux valeurs θ₃ = +arccos(...) et θ₃ = −arccos(...) sont toutes deux solutions. Géométriquement, ces deux solutions correspondent à deux façons distinctes de plier le tibia par rapport au prolongement du fémur : d'un côté ou de l'autre. Les deux configurations produisent la même distance r entre la hanche et le pied — c'est précisément pourquoi la loi des cosinus, qui ne dépend que de r, L1 et L2, ne peut pas les distinguer.
Ce n'est pas une ambiguïté de quadrant (comme pour un atan2) mais un choix binaire de côté du pli.

Ainsi, on a : 

$$\theta_3=-Arccos(\frac{r²-L_1²-L_2²}{2.L_1.L_2})$$

>[Note]
>Ce résultat s'obtient grâce à l'utilisation de la loi de cosinus.

### 4.3 Calcul de θ2

On a : 
$$\begin{cases}
x= L_1.sin(\theta_2)+L_2.sin(\theta_2+\theta_3) \\
A=L_1.cos(\theta_2)+L_2.cos(\theta_2+\theta_3) 
\end{cases}$$ 
On développe le système en utilisant les formules de trigonométrie $cos(\theta_2 + \theta_3)$ et $sin(\theta_2 + \theta_3)$.

On obtient :
$$\begin{cases}
x= (L_1+L2.cos(\theta_3)).sin(\theta_2)+L_2.sin(\theta_3).cos(\theta_2) \\
A=(L_1+L2.cos(\theta_3)).cos(\theta_2)-L_2.sin(\theta_3).cos(\theta_2)
\end{cases}$$ 
On pose : 

$U=L_1 +L_2.cos(\theta_3)$

et 

$V=L_2.sin(\theta_3)$

On résout le système : 
$$\begin{cases}
x= U.sin(\theta_2)+V.cos(\theta_2) \\
A=U.cos(\theta_2)-V.sin(\theta_2)
\end{cases}$$ 
pour touver les expressions algébrique de $cos(\theta_2)$ et $\sin(theta_2)$ en fonctoin de x, A, U et V.

On trouve : 

$$\begin{cases}
cos(\theta_2)= \frac{A.U+V.x}{U²+V²}\\
sin(\theta_2)=\frac{x.U-V.A}{U²+V²}
\end{cases}$$ 

Ainsi, on a : 

$$\theta_2=atan^2(\frac{x.U-V.A}{A.U+V.x})$$

>[!Warning]
>A dépend de $\theta_2$ et $\theta_3$, or $\theta_1$ dépend de A et $\theta_2$ dépend de $\theta_1$, donc il faut trouver $A $ indépendant de $\theta_2$.
Ainsi, en normalisant $P_{(Y,Z)}$, on obtient $A=\sqrt{y²+z²-L_0²}$

### 4.4 Chaîne de calcul IK Jambe

$$X_{(X,Y,Z)} => A => \theta_1 => r => \theta_3 => \theta_2$$

### 4.5 Cas particuliers

Les différents cas particuliers traités par la cinématique inverse sont résumés dans le tableau suivant :

| Condition                                      | Interprétation                                                  | Comportement algorithmique                                                |                                  |                                 |
| ---------------------------------------------- | --------------------------------------------------------------- | ------------------------------------------------------------------------- | -------------------------------- | ------------------------------- |
| $y^2+z^2<L_0^2$                                | La cible est incompatible avec l’offset de hanche               | `return std::nullopt`                                                     |                                  |                                 |
| $r>L_1+L_2$                                    | La cible est trop éloignée                                      | `return std::nullopt`                                                     |                                  |                                 |
| $\lvert L_1-L_2 \rvert$                                                                                                                         | La cible est trop proche         | `return std::nullopt`           |
| $r=L_1+L_2$                                    | La patte est entièrement tendue                                 | Cible acceptée, $\theta_3=0$                                              |                                  |                                 |
| $r=\left                                       | L_1-L_2\right                                                   | $                                                                         | La patte est entièrement repliée | Cible acceptée, $\theta_3=-\pi$ |
| $D\notin[-1,1]$ à cause des erreurs numériques | Dépassement faible dû aux nombres flottants                     | Borner $D$ dans $[-1,1]$ avant `acos`                                     |                                  |                                 |
| Angles hors des limites mécaniques             | Cible géométriquement atteignable, mais configuration interdite | `inverse()` retourne les angles, puis `isWithinLimits()` retourne `false` |                                  |                                 |

avec :

$$
r=\sqrt{x^2+A^2}
$$

et :

$$
D=\frac{r^2-L_1^2-L_2^2}{2L_1L_2}
$$

Le bornage numérique de $D$ est effectué uniquement après avoir vérifié que la cible appartient bien à l’espace atteignable.

