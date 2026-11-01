# 🦾 Bionic Arm

My own robotic arm, built from scratch, starting with the math.

The goal: give the arm a target (a red dot on the ground) and let it work out
**by itself** how to bend each joint to grab it, coming from above like a real
pick-and-place arm.

```
        P2 ●━━━━━━━━● P3
          ╱           ╲
     L1  ╱     L2      ╲ L3
        ╱                ● P4  ← target
       ╱
  P1 ●
  (base)
```

## Status

| Step | State |
|------|-------|
| Forward kinematics (angles → position) | ✅ Done |
| Draw the arm with matplotlib | ✅ Done |
| Inverse kinematics, 3 links (position → angles) | 🚧 In progress |
| Choose the best approach angle (grab from above) | 🚧 In progress |
| Reachability check (is the target in range?) | 🚧 In progress |
| 3D arm | 🔜 Planned |
| Real hardware (servos) | 🔜 Planned |

## How it works

The arm has 3 links (`L1 = 10`, `L2 = 5`, `L3 = 3`) in a 2D plane.

1. **Pick an approach angle θ** for the last link (the "finger").
2. **Find the wrist (P3)**: step back from the target along θ by `L3`.
3. **Solve the triangle P1-P2-P3** with the law of cosines, which gives the
   shoulder and elbow angles.
4. **Deduce the last angle**: the three joint angles must add up to θ.
5. **Check**: feeding the angles back into forward kinematics must land
   exactly on the target.

Tools used: Pythagoras, SOH CAH TOA, law of cosines.

## Run it

```bash
python3 -m venv robot-env
source robot-env/bin/activate
pip install matplotlib
python arms3.py
```

The arm is drawn and saved as `arm3.png`.

## Files

| File | Role |
|------|------|
| `arms3.py` | Kinematics, solver and drawing for the 3-link arm |
| `arm3.png` | Latest rendering of the arm |

## Roadmap

- [x] Understand the geometry on paper
- [x] Forward kinematics
- [ ] Inverse kinematics for 3 links
- [ ] Automatic approach angle, from above
- [ ] Handle unreachable targets cleanly
- [ ] Go 3D
- [ ] Build the physical arm

## Notes to self

- Angles in `forward` are **relative** to the previous link; the angle θ of
  the finger is **absolute**.
- Degrees for the math, radians for `math.cos` and `math.sin`. Convert in one
  single place.
- Near full extension, `acos` can receive 1.0000000002. Always clamp its input.

---

Made at Epitech, one triangle at a time.
