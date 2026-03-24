#!/usr/bin/env python3
"""Render 4 CF2x drones in a square formation in MuJoCo with motion-blurred props."""
import os
import mujoco
import numpy as np
from PIL import Image, ImageFilter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MUJOCO_DIR = os.path.join(SCRIPT_DIR, '..', 'crazyflie-firmware', 'tools',
                          'crazyflie-simulation', 'simulator_files', 'mujoco')

# Square formation positions and yaw angles (each drone faces a different direction)
DRONES = [
    {'pos': '-0.09 -0.09 0', 'yaw': 0},
    {'pos': '0.09 -0.09 0',  'yaw': 0},
    {'pos': '-0.09 0.09 0',  'yaw': 0},
    {'pos': '0.09 0.09 0',   'yaw': 0},
]

NUM_BLUR_FRAMES = 20
SWEEP_ANGLE = 2.0 * np.pi / 3.0  # 120° partial rotation


def make_drone_body(idx, pos, yaw):
    offsets = [0.0, np.pi/4, np.pi/2, 3*np.pi/4]
    return f"""
    <body name="cf{idx}" pos="{pos}" euler="0 0 {yaw}">
      <geom type="mesh" mesh="pcb" material="wire"/>
      <geom type="mesh" mesh="motors" material="wire"/>
      <geom type="mesh" mesh="motor_holder" material="wire"/>
      <geom type="mesh" mesh="battery" material="wire"/>
      <geom type="mesh" mesh="battery_holder" material="wire"/>
      <geom type="mesh" mesh="connectors" material="wire"/>
      <geom type="mesh" mesh="pins" material="wire"/>
      <geom type="mesh" mesh="led" material="wire"/>
      <body pos="0.0325 -0.0325 0.015" euler="0 0 45">
        <joint name="prop{idx}_0" type="hinge" axis="0 0 1" limited="false"/>
        <geom type="mesh" mesh="propL" material="prop_wire"/>
      </body>
      <body pos="-0.0325 -0.0325 0.015" euler="0 0 135">
        <joint name="prop{idx}_1" type="hinge" axis="0 0 1" limited="false"/>
        <geom type="mesh" mesh="propR" material="prop_wire"/>
      </body>
      <body pos="-0.0325 0.0325 0.015" euler="0 0 225">
        <joint name="prop{idx}_2" type="hinge" axis="0 0 1" limited="false"/>
        <geom type="mesh" mesh="propL" material="prop_wire"/>
      </body>
      <body pos="0.0325 0.0325 0.015" euler="0 0 315">
        <joint name="prop{idx}_3" type="hinge" axis="0 0 1" limited="false"/>
        <geom type="mesh" mesh="propR" material="prop_wire"/>
      </body>
    </body>"""


def make_mjcf():
    bodies = ""
    for i, d in enumerate(DRONES):
        bodies += make_drone_body(i, d['pos'], d['yaw'])

    return f"""
<mujoco>
  <compiler meshdir="{MUJOCO_DIR}/drone-models/drone_models/data/assets/cf2x"/>
  <asset>
    <mesh name="pcb" file="cf2x_pcb.stl" scale="0.001 0.001 0.001"/>
    <mesh name="motors" file="cf2xT_motors.stl" scale="0.001 0.001 0.001"/>
    <mesh name="motor_holder" file="cf2x_motor-holder.stl" scale="0.001 0.001 0.001"/>
    <mesh name="battery" file="cf2x_battery.stl" scale="0.001 0.001 0.001"/>
    <mesh name="battery_holder" file="cf2x_battery-holder.stl" scale="0.001 0.001 0.001"/>
    <mesh name="connectors" file="cf2x_connectors.stl" scale="0.001 0.001 0.001"/>
    <mesh name="pins" file="cf2x_connector-pins.stl" scale="0.001 0.001 0.001"/>
    <mesh name="led" file="cf_led-diffusor.stl" scale="0.001 0.001 0.001"/>
    <mesh name="propL" file="../cf21B/cf21B_PropL.stl" scale="0.001 0.001 0.001"/>
    <mesh name="propR" file="../cf21B/cf21B_PropR.stl" scale="0.001 0.001 0.001"/>
    <material name="wire" rgba="0.0 0.86 0.78 0.4"/>
    <material name="prop_wire" rgba="0.0 0.92 0.85 0.95"/>
  </asset>
  <worldbody>
    <!-- Global overhead light -->
    <light pos="0 0 0.4" dir="0 0 -1" diffuse="0.0 0.7 0.65" specular="0.0 0.3 0.3"/>
    <!-- Per-drone local lights -->
    <light pos="-0.09 -0.09 0.15" dir="0 0 -1" diffuse="0.0 0.4 0.35" specular="0.0 0.15 0.15"/>
    <light pos="0.09 -0.09 0.15" dir="0 0 -1" diffuse="0.0 0.4 0.35" specular="0.0 0.15 0.15"/>
    <light pos="-0.09 0.09 0.15" dir="0 0 -1" diffuse="0.0 0.4 0.35" specular="0.0 0.15 0.15"/>
    <light pos="0.09 0.09 0.15" dir="0 0 -1" diffuse="0.0 0.4 0.35" specular="0.0 0.15 0.15"/>
    {bodies}
  </worldbody>
</mujoco>
"""


def make_transparent(pixels):
    img = Image.fromarray(pixels).convert('RGBA')
    data_px = np.array(img)
    bg = data_px[0, 0, :3]
    mask = np.all(np.abs(data_px[:, :, :3].astype(int) - bg.astype(int)) < 25, axis=2)
    data_px[mask] = [0, 0, 0, 0]
    return Image.fromarray(data_px, 'RGBA')


def main():
    model = mujoco.MjModel.from_xml_string(make_mjcf())
    data = mujoco.MjData(model)

    width, height = 300, 300
    renderer = mujoco.Renderer(model, height=height, width=width)

    # Camera looking at the formation from a slight angle
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.005]
    camera.distance = 0.36
    camera.azimuth = 180
    camera.elevation = -90  # top-down view, 2D plane

    scene_option = mujoco.MjvOption()
    scene_option.frame = mujoco.mjtFrame.mjFRAME_NONE
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = False

    # Prop angle offsets per motor (so blades don't all align)
    prop_offsets = [0.0, np.pi/4, np.pi/2, 3*np.pi/4]
    n_props = model.njnt  # 4 drones x 4 props = 16

    print(f'Rendering {NUM_BLUR_FRAMES} frames for motion blur ({n_props} joints)...')
    accumulator = np.zeros((height, width, 4), dtype=np.float64)

    for i in range(NUM_BLUR_FRAMES):
        base_angle = (SWEEP_ANGLE / NUM_BLUR_FRAMES) * i
        for drone_idx in range(4):
            for prop_idx in range(4):
                j = drone_idx * 4 + prop_idx
                data.qpos[j] = base_angle + prop_offsets[prop_idx]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera, scene_option=scene_option)
        pixels = renderer.render().copy()
        frame = make_transparent(pixels)
        accumulator += np.array(frame).astype(np.float64)

    averaged = (accumulator / NUM_BLUR_FRAMES).astype(np.float64)
    # Boost prop blur alpha
    averaged[:, :, 3] = np.clip(averaged[:, :, 3] * 3.0, 0, 255)
    img = Image.fromarray(averaged.astype(np.uint8), 'RGBA')

    # Glow
    glow = img.copy().filter(ImageFilter.GaussianBlur(radius=3))
    glow_px = np.array(glow)
    glow_px[:, :, :3] = np.clip(glow_px[:, :, :3].astype(int) * 2, 0, 255).astype(np.uint8)
    glow_px[:, :, 3] = np.clip(glow_px[:, :, 3].astype(int), 0, 160).astype(np.uint8)
    glow = Image.fromarray(glow_px, 'RGBA')

    result = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    result = Image.alpha_composite(result, glow)
    result = Image.alpha_composite(result, img)

    out_path = os.path.join(SCRIPT_DIR, 'crazysim_drone_render.png')
    result.save(out_path)
    print(f'Saved to {out_path} ({width}x{height})')


if __name__ == '__main__':
    main()
