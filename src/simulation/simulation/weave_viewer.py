"""Single-threaded MuJoCo display for the weave controller."""
from contextlib import contextmanager
import os
import platform

import mujoco as mj


@contextmanager
def weave_viewer(model, lookat, distance, azimuth, elevation):
    import glfw

    wsl = "microsoft" in platform.release().lower()
    if wsl:
        os.environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        os.environ["GALLIUM_DRIVER"] = "d3d12"
        os.environ.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")

    if not glfw.init():
        raise RuntimeError("GLFW could not initialize the display")
    window = context = None
    try:
        glfw.default_window_hints()
        window = glfw.create_window(1100, 720, "FORGE | Coordinated weave", None, None)
        if not window:
            raise RuntimeError("Could not create the FORGE weave window")
        glfw.make_context_current(window)
        from OpenGL.GL import GL_RENDERER, glGetString

        renderer = (glGetString(GL_RENDERER) or b"unknown").decode(errors="replace")
        print(f"OpenGL renderer: {renderer}", flush=True)
        if wsl and (not renderer.startswith("D3D12 (") or "basic render" in renderer.lower() or "warp" in renderer.lower()):
            raise RuntimeError(f"WSLg hardware rendering unavailable ({renderer}). Check the Windows GPU driver and WSLg.")
        glfw.swap_interval(1)
        context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
        scene = mj.MjvScene(model, maxgeom=3000)
        camera, option = mj.MjvCamera(), mj.MjvOption()
        camera.lookat[:] = lookat
        camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
        previous = list(glfw.get_cursor_pos(window))

        def drag(window, x, y):
            height = max(1, glfw.get_window_size(window)[1])
            dx, dy = (x - previous[0]) / height, (y - previous[1]) / height
            previous[:] = x, y
            if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT):
                mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ROTATE_V, dx, dy, camera)
            elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT):
                mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_V, dx, dy, camera)

        glfw.set_cursor_pos_callback(window, drag)
        glfw.set_scroll_callback(window, lambda _, x, y:
            mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0, -.05*y, camera))

        def render(data, draw, labels, values):
            glfw.poll_events()
            if glfw.window_should_close(window) or glfw.get_key(window, glfw.KEY_ESCAPE):
                return False
            width, height = glfw.get_framebuffer_size(window)
            if width and height:
                viewport = mj.MjrRect(0, 0, width, height)
                mj.mjv_updateScene(model, data, option, None, camera, mj.mjtCatBit.mjCAT_ALL, scene)
                draw(scene)
                mj.mjr_render(viewport, scene, context)
                mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT,
                               viewport, labels, values, context)
                glfw.swap_buffers(window)
            return True

        yield render
    finally:
        if context is not None:
            context.free()
        if window:
            glfw.destroy_window(window)
        glfw.terminate()
