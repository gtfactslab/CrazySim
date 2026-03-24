#!/usr/bin/env python3
"""Render the CF2x drone model from MuJoCo as a wireframe for the CrazySim logo."""
import os
import mujoco
import numpy as np
from PIL import Image, ImageFilter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MUJOCO_DIR = os.path.join(SCRIPT_DIR, '..', 'crazyflie-firmware', 'tools',
                          'crazyflie-simulation', 'simulator_files', 'mujoco')

# Minimal MJCF — all parts same cyan wireframe color
MJCF = f"""
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
  </asset>
  <worldbody>
    <light pos="0.1 -0.15 0.25" dir="-0.3 0.5 -0.7" diffuse="0.0 0.9 0.8" specular="0.0 0.5 0.5"/>
    <light pos="-0.1 0.1 0.2" dir="0.3 -0.3 -0.7" diffuse="0.0 0.5 0.45" specular="0.0 0.2 0.2"/>
    <body name="cf2x" pos="0 0 0">
      <geom type="mesh" mesh="pcb" material="wire"/>
      <geom type="mesh" mesh="motors" material="wire"/>
      <geom type="mesh" mesh="motor_holder" material="wire"/>
      <geom type="mesh" mesh="battery" material="wire"/>
      <geom type="mesh" mesh="battery_holder" material="wire"/>
      <geom type="mesh" mesh="connectors" material="wire"/>
      <geom type="mesh" mesh="pins" material="wire"/>
      <geom type="mesh" mesh="led" material="wire"/>
      <body pos="0.0325 -0.0325 0.015" euler="0 0 45">
        <geom type="mesh" mesh="propL" material="wire"/>
      </body>
      <body pos="-0.0325 -0.0325 0.015" euler="0 0 135">
        <geom type="mesh" mesh="propR" material="wire"/>
      </body>
      <body pos="-0.0325 0.0325 0.015" euler="0 0 225">
        <geom type="mesh" mesh="propL" material="wire"/>
      </body>
      <body pos="0.0325 0.0325 0.015" euler="0 0 315">
        <geom type="mesh" mesh="propR" material="wire"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def main():
    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    width, height = 400, 300
    renderer = mujoco.Renderer(model, height=height, width=width)

    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.003]
    camera.distance = 0.14
    camera.azimuth = 150
    camera.elevation = -30

    # Enable wireframe rendering
    scene_option = mujoco.MjvOption()
    scene_option.frame = mujoco.mjtFrame.mjFRAME_NONE
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = False

    renderer.update_scene(data, camera=camera, scene_option=scene_option)

    # Render solid pass
    solid_pixels = renderer.render().copy()

    # Render wireframe pass
    scene = renderer._scene
    for i in range(scene.ngeom):
        scene.geoms[i].dataid = -1  # force wireframe by removing mesh data

    # Instead, use MuJoCo's built-in wireframe flag
    renderer.update_scene(data, camera=camera, scene_option=scene_option)

    pixels = renderer.render()

    # Convert to image
    img = Image.fromarray(pixels)
    img = img.convert('RGBA')
    data_px = np.array(img)

    # Make background transparent
    bg = data_px[0, 0, :3]
    mask = np.all(np.abs(data_px[:, :, :3].astype(int) - bg.astype(int)) < 15, axis=2)
    data_px[mask] = [0, 0, 0, 0]

    # Apply a subtle glow effect: duplicate the drone, blur it, composite underneath
    img = Image.fromarray(data_px, 'RGBA')

    # Create glow layer
    glow = img.copy()
    glow = glow.filter(ImageFilter.GaussianBlur(radius=4))
    glow_px = np.array(glow)
    # Boost glow brightness
    glow_px[:, :, :3] = np.clip(glow_px[:, :, :3].astype(int) * 2, 0, 255).astype(np.uint8)
    glow_px[:, :, 3] = np.clip(glow_px[:, :, 3].astype(int) * 1, 0, 180).astype(np.uint8)
    glow = Image.fromarray(glow_px, 'RGBA')

    # Composite: glow behind, sharp on top
    result = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    result = Image.alpha_composite(result, glow)
    result = Image.alpha_composite(result, img)

    out_path = os.path.join(SCRIPT_DIR, 'crazysim_drone_render.png')
    result.save(out_path)
    print(f'Saved to {out_path} ({width}x{height})')


if __name__ == '__main__':
    main()
