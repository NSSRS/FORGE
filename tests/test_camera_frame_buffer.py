"""Latest-frame delivery remains bounded while a renderer falls behind."""
import unittest
import source_paths  # noqa: F401
from stereo_cam.frame_buffer import FrameBuffer


class FrameBufferTests(unittest.TestCase):
    def test_slow_renderer_gets_newest_image_and_all_recent_metrics(self):
        buffer = FrameBuffer()
        frames = [object() for _ in range(60)]
        for i, frame in enumerate(frames):
            buffer.put(frame, i/60)
        latest, samples = buffer.take()
        self.assertIs(latest[0], frames[-1])
        self.assertEqual(len(samples), 60)
        self.assertEqual(buffer.take(), (None, []))

    def test_stalled_gui_does_not_create_unbounded_history(self):
        buffer = FrameBuffer()
        for i in range(5000):
            buffer.put(i, float(i))
        latest, samples = buffer.take()
        self.assertEqual(latest[0], 4999)
        self.assertEqual(len(samples), 4096)

    def test_receiver_can_continue_while_previous_image_is_in_use(self):
        buffer = FrameBuffer()
        buffer.put('old', 1.)
        old, _ = buffer.take()
        buffer.put('new', 2.)
        new, _ = buffer.take()
        self.assertEqual(old[0], 'old')
        self.assertEqual(new[0], 'new')
