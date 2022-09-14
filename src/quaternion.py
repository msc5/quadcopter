import torch

from . import types as tt
from . import vector as V


def from_theta(e: tt.Vector3, theta: tt.Float) -> tt.Quaternion:
    v = e * (theta[..., None] / 2).sin()
    s = (theta[..., None] / 2).cos()
    return torch.cat([v, s], dim=-1)


def psi(q: tt.Quaternion) -> tt.Matrix:
    batch = q.shape[:-1]
    qv = q[..., 0:3]
    q4 = q[..., 3]
    qcross = V.cross(qv)
    a = q4[..., None, None] * V.batch(torch.eye(3), batch) - qcross
    b = - qv[..., None, :]
    return torch.cat([a, b], dim=-2)


def xi(q: tt.Quaternion) -> tt.Matrix:
    batch = q.shape[:-1]
    qv = q[..., 0:3]
    q4 = q[..., 3]
    qcross = V.cross(qv)
    a = q4[..., None, None] * V.batch(torch.eye(3), batch) + qcross
    b = - qv[..., None, :]
    return torch.cat([a, b], dim=-2)


def cross(q: tt.Quaternion) -> tt.Matrix:
    return torch.cat([psi(q), q[..., None]], dim=-1)


def dot(q: tt.Quaternion) -> tt.Matrix:
    return torch.cat([xi(q), q[..., None]], dim=-1)


def A(q: tt.Quaternion) -> tt.Matrix:
    return xi(q).transpose(-1, -2) @ psi(q)


if __name__ == "__main__":

    from .cli import console

    v = torch.rand(5, 3)
    theta = torch.rand(5)

    rot = from_theta(v, theta)
    console.log('Quaternion')
    console.log(rot, rot.shape)

    rot_xi = xi(rot)
    console.log('Xi')
    console.log(rot_xi, rot_xi.shape)

    rot_cross = cross(rot)
    console.log('Cross')
    console.log(rot_cross, rot_cross.shape)

    rot_A = A(rot)
    console.log('A')
    console.log(rot_A, rot_A.shape)
