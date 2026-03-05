#include "tarea2/tang.hpp"
#include <cmath>

TangentialOutput compute_tangential_escape(
    double e_theta,
    double d,
    double A_front,
    double A_left,
    double A_right,
    double k_tan)
{
    TangentialOutput out;

    // 1. Detectar si el objetivo está bloqueado
    bool objetivo_delante = std::abs(e_theta) < 0.5;   // ±30º
    bool obstaculo_frontal = A_front > 0.15;           // ajustable

    if (!(objetivo_delante && obstaculo_frontal)) {
        return out; // no activo
    }

    out.active = true;

    // 2. Determinar el lado más despejado
    bool seguir_por_derecha = (A_right < A_left);

    // 3. Generar giro tangencial
    double base = k_tan * A_front;

    if (seguir_por_derecha)
        out.w_tang = +base;   // seguir borde por la derecha
    else
        out.w_tang = -base;   // seguir borde por la izquierda

    return out;
}
