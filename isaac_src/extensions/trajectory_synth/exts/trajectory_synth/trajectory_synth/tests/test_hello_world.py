import omni.kit.test

# Extnsion for writing UI tests (simulate UI interaction)
from omni.kit import ui_test

import trajectory_synth


class Test(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_hello_public_function(self):
        result = trajectory_synth.some_public_function(4)
        self.assertEqual(result, 256)

    async def test_window_button(self):
        # Find a label in our window
        label = ui_test.find("My Window//Frame/**/Label[*]")

        # Find buttons in our window
        add_button = ui_test.find("My Window//Frame/**/Button[*].text=='Add'")
        reset_button = ui_test.find("My Window//Frame/**/Button[*].text=='Reset'")

        # Click reset button
        await reset_button.click()
        self.assertEqual(label.widget.text, "empty")

        await add_button.click()
        self.assertEqual(label.widget.text, "count: 1")

        await add_button.click()
        self.assertEqual(label.widget.text, "count: 2")
