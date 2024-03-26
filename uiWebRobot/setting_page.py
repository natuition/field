from unicodedata import category
from typing import Callable
from flask_socketio import SocketIO
import fileinput
import pwd
import grp
import os


class ItemInterface:

    def __init__(self, id: str, label: str, callback: Callable = None):
        self._id: str = id
        self._label: str = label
        self._callback: Callable = callback
        self._html_id = f"{self.__class__.__name__}-{self._id}"

    def get_id(self) -> str:
        return self._id

    def callback(self, new_value) -> None:
        if self._callback is not None:
            self._callback(new_value)

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        raise Exception("NotImplementedException")


class RadioButton(ItemInterface):

    def __init__(self, id: str, label: str, callback: Callable):
        super().__init__(id, label, callback)
        self.__checked: str = ""

    def set_checked(self, checked: bool):
        if checked:
            self.__checked = "checked"
        else:
            self.__checked = ""

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        return f"""<div class="checkbox" style="margin-top: 0px !important;">
                        <label class="checkbox__item">
                            <p class="checkbox__item--title unselectable">
                                {ui_languages[self._label][ui_language]}
                            </p>
                            <input type="checkbox" class="new_value" id="{self._html_id}" {self.__checked}>
                            <span class="checkmark">
                                <svg xmlns="http://www.w3.org/2000/svg" width="22" height="22" fill="#47a857" class="bi bi-check" viewBox="0 0 16 16">
                                    <circle cx="8.5" cy="8" r="4" fill="#47a857" />
                                </svg>
                            </span>
                        </label>
                    </div>"""


class Checkbox(ItemInterface):

    def __init__(self, id: str, label: str, callback: Callable):
        super().__init__(id, label, callback)
        self.__checked: str = ""

    def set_checked(self, checked: bool):
        if checked:
            self.__checked = "checked"
        else:
            self.__checked = ""

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        return f"""<div class="checkbox">
                        <label class="checkbox__item">
                            <p class="checkbox__item--title unselectable">
                                {ui_languages[self._label][ui_language]}
                            </p>
                            <input type="checkbox" class="new_value" id="{self._html_id}" {self.__checked}>
                            <span class="checkmark">
                                <svg xmlns="http://www.w3.org/2000/svg" width="22" height="22" fill="#47a857" class="bi bi-check" viewBox="0 0 16 16">
                                    <path d="M10.97 4.97a.75.75 0 0 1 1.07 1.05l-3.99 4.99a.75.75 0 0 1-1.08.02L4.324 8.384a.75.75 0 1 1 1.06-1.06l2.094 2.093 3.473-4.425a.267.267 0 0 1 .02-.022z"/>
                                </svg>
                            </span>
                        </label>
                    </div>"""


class Selector(ItemInterface):

    def __init__(self, id: str, label: str, callback: Callable):
        super().__init__(id, label, callback)
        self._content_list: list = None
        self._default_content_index: int = None
        self._choose_description: str = None

    def set_content_list(self, content_list: list, default_content):
        self._content_list: list = content_list
        self._default_content_index: int = content_list.index(default_content)

    def set_choose_description(self, choose_description: str):
        self._choose_description: str = choose_description

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        content_list_html = [f"<option value='{content}'"+(f" selected='selected'>{content}</option>" if count ==
                                                           self._default_content_index else f">{content}</option>") for count, content in enumerate(self._content_list)]
        content_list_html = "\n".join(content_list_html)
        return f"""<div class="checklist__select">
                        <div class="checklist__select--title">
                            {ui_languages[self._label][ui_language]}
                        </div>
                        <div class="checklist__select--role">
                            <select id="{self._html_id}" class="new_value">
                                <option value='{ui_languages[self._choose_description][ui_language]}' disabled>{ui_languages[self._choose_description][ui_language]}</option>
                                {content_list_html}
                            </select>
                            <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="#9ca2b4" class="bi bi-caret-down-fill position-absolute" viewBox="0 0 16 16">
                                <path d="M7.247 11.14L2.451 5.658C1.885 5.013 2.345 4 3.204 4h9.592a1 1 0 0 1 .753 1.659l-4.796 5.48a1 1 0 0 1-1.506 0z"></path>
                            </svg>
                        </div>
                    </div>"""


class Slider(ItemInterface):

    def __init__(self, id: str, label: str, callback: Callable):
        super().__init__(id, label, callback)
        self._min: float = None
        self._max: float = None
        self._step: float = None
        self._default: float = None
        self.__span_id = f"{self._html_id}"
        self.__range_id = f"{self._html_id}_range"

    def set_number_parameters(self, min: float, max: float, step: float, default: float):
        self._min: float = min
        self._max: float = max
        self._step: float = step
        self._default: float = default

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        return f"""<div class="ruler__main--assets" style="margin-top:10px;">
                        <div class="ruler__assets--title">
                            <p>{ui_languages[self._label][ui_language]}</p>
                            <span style="text-align: center; width: auto !important;" class="no_format new_value slider" id="{self.__span_id}">{self._default}</span>
                        </div>
                        <div class="ruler__assets--line">
                            <input type="range" min="{self._min}" max="{self._max}" value="{self._default}" step="{self._step}" id="{self.__range_id}" oninput="range('{self.__range_id}', '{self.__span_id}')"">
                        </div>
                    </div>
                    <script defer>
                        range("{self.__range_id}", "{self.__span_id}")
                    </script>"""


class Button(ItemInterface):

    GREEN = ["71", "168", "87"]
    RED = ["234", "40", "40"]
    WHITE = ["255", "255", "255"]

    def __init__(self, id: str, label: str):
        super().__init__(id, label)
        self._color_rgb: list[str] = None
        self.border = ""
        self._on_click_fct_name = ""

    def set_color(self, color_rgb: 'list[str]'):
        self._color_rgb: list[str] = color_rgb
        if color_rgb == self.WHITE:
            self.border = "border-color: rgb(71, 168, 87); border-width: 1px; border-style: solid; color: #778493;"

    def set_on_click_fct_name(self, fct_name):
        self._on_click_fct_name = fct_name

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        return f""" <button name="{self._html_id}" id="{self._html_id}" onclick="{self._on_click_fct_name}" class="setting_btn unselectable" style="{self.border} background-color: rgb({", ".join(self._color_rgb)})!important; box-shadow: 0px 4px 14px 0px rgba({", ".join(self._color_rgb)}, 0.6)!important;">
                        <span class="submit">{ui_languages[self._label][ui_language]}</span>
                        <span class="loading"><i class="fas fa-sync-alt fa-spin"></i></span>
                        <span class="check"><i class="fas fa-check scale"></i></span>
                    </button>"""


class RadioButtonGroup(ItemInterface):

    def __init__(self, id: str, label: str):
        super().__init__(id, label)
        self._column_number: int = None
        self._radio_button_list: list[RadioButton] = None

    def set_radio_button_list(self, radio_button_list: 'list[RadioButton]'):
        self._radio_button_list: list[RadioButton] = radio_button_list

    def set_column_number(self, column_number: int):
        self._column_number = column_number

    def get_items(self) -> 'list[ItemInterface]':
        return self._radio_button_list

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        radio_button_html = ""
        for radio_btn in self._radio_button_list:
            radio_button_html += radio_btn.generate_html(
                ui_languages, ui_language)+"\n"
        return f""" <div class="checklist__select--title" style="margin-bottom: 0px !important;">
                        {ui_languages[self._label][ui_language]}
                    </div>
                    <div class="radio_ckeckbox {self.__class__.__name__} {self._id}" style="grid-template-columns:{(str(100/self._column_number)+"% ")*self._column_number};">
                        {radio_button_html}
                    </div>"""


class ButtonGroup(ItemInterface):

    def __init__(self, id: str, label: str, button_list: 'list[Button]'):
        super().__init__(id, label)
        self._button_list: list[Button] = button_list

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        button_html = ""
        for btn in self._button_list:
            button_html += btn.generate_html(ui_languages, ui_language)+"\n"
        return f""" <div class="setting--button">
                        {button_html}
                    </div>"""


class Category(ItemInterface):

    def __init__(self, id: str, label: str):
        super().__init__(id, label)
        self.__items_list: list[ItemInterface] = list()

    def add_item(self, item: ItemInterface):
        self.__items_list.append(item)

    def add_items(self, items: 'list[ItemInterface]'):
        self.__items_list.extend(items)

    def get_items(self) -> 'list[ItemInterface]':
        return self.__items_list

    def __get_string_items(self, ui_languages: dict, ui_language: str) -> str:
        str_items = ""
        for item in self.__items_list:
            str_items += item.generate_html(ui_languages, ui_language) + "\n"
        return str_items

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        return f""" <section class="setting__category">
                        <h1 class="setting__category--title unselectable">
                            {ui_languages[self._label][ui_language]}
                        </h1>
                        {self.__get_string_items(ui_languages, ui_language)}
                    </section>"""


class SettingGenerator():

    def __init__(self):
        self.__items_list: list[ItemInterface] = list()

        self.__btn_save = Button("save", "Save")
        self.__btn_save.set_color(Button.GREEN)
        self.__btn_save.set_on_click_fct_name("save_setting()")

        self.__btn_cancel = Button("cancel", "Cancel")
        self.__btn_cancel.set_color(Button.RED)
        self.__btn_cancel.set_on_click_fct_name("go_to_page('/')")

        self.__btn_group_save_cancel = ButtonGroup(
            "btn_save_cancel", "Save Cancel button", [self.__btn_save, self.__btn_cancel])

    def add_item(self, item: ItemInterface):
        self.__items_list.append(item)

    def add_items(self, items: 'list[ItemInterface]'):
        self.__items_list.extend(items)

    def get_items(self) -> 'list[ItemInterface]':
        return self.__items_list

    def __get_string_items(self, ui_languages: dict, ui_language: str) -> str:
        str_items = ""
        for item in self.__items_list:
            str_items += item.generate_html(ui_languages, ui_language) + "\n"
        return str_items + self.__btn_group_save_cancel.generate_html(ui_languages, ui_language)

    def generate_html(self, ui_languages: dict, ui_language: str) -> str:
        return f""" <section class="setting">
                        {self.__get_string_items(ui_languages, ui_language)}
                    </section>"""


class SettingPageManager:

    def __init__(self, socket_io: SocketIO, ui_languages: dict, config, reload_config: Callable):
        self.__socket_io: SocketIO = socket_io
        self.__ui_languages = ui_languages
        self.__socket_io.on_event(
            'data', self.__save_event, namespace='/save_setting')
        self.__reload_config = reload_config
        self.__config = config
        self.__setting_generator = SettingGenerator()
        self.__change_config_value = dict()

    def __add_to_setting_class_whitout_group(self, items, setting_class_whitout_group: 'list[ItemInterface]'):
        if isinstance(items, (Category, RadioButtonGroup)):
            for _items in items.get_items():
                self.__add_to_setting_class_whitout_group(
                    _items, setting_class_whitout_group)
        elif not isinstance(items, ButtonGroup):
            setting_class_whitout_group.append(items)

    def __save_event(self, save_data):
        class_new_value = dict()
        for repr_class, value in save_data.items():
            class_name = repr_class.split("-")[0]
            class_id = repr_class.split("-")[1]
            class_new_value[(class_name, class_id)] = value

        setting_class_whitout_group: list[ItemInterface] = list()
        for setting_class in self.__setting_generator.get_items():
            self.__add_to_setting_class_whitout_group(
                setting_class, setting_class_whitout_group)

        for setting_class in setting_class_whitout_group:
            setting_class.callback(
                class_new_value[(setting_class.__class__.__name__), setting_class.get_id()])

        self.__applyConfigValue()

        self.__reload_config()

        self.__socket_io.emit(
            'save_finish', {}, namespace='/save_setting', broadcast=True)

    def __changeConfigDict(self, current_dict: dict, path: str, key, value):
        current_dict[key] = value
        self.__changeConfigValue(path, current_dict)

    def __changeConfigValue(self, path: str, value, is_str: bool = False):
        self.__change_config_value[path] = (value, is_str)

    def __applyConfigValue(self):
        if self.__change_config_value:
            with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
                for line in file:
                    find = False
                    for path, value_is_str in self.__change_config_value.items():
                        if path in line:
                            if not line.split(path)[0]:
                                if value_is_str[1]:
                                    print(path + " = \"" +
                                          str(value_is_str[0]), end='"\n')
                                else:
                                    print(path + " = " +
                                          str(value_is_str[0]), end='\n')
                                find = True
                    if not find:
                        print(line, end='')
            uid = pwd.getpwnam("violette").pw_uid
            gid = grp.getgrnam("violette").gr_gid
            os.chown("../config/config.py", uid, gid)

    def __get_ia_list(self, dir_path) -> list:
        ia_list = []
        for file in os.listdir(dir_path):
            if file.endswith(".trt"):
                ia_list.append(file.split(".trt")[0])
        return ia_list

    def __set_ia_in_config(self, mod_name: str, ia_name_with_resolution: str):
        """
        Becareful the format of ia_name_with_resolution is : YXXX_YYY_ZZZ.trt
        Were XXX is name of ia, YYY is x resolution and ZZZ is Y resolution.
        """
        ia_name = ia_name_with_resolution.split("_")[0]
        resolution = tuple([int(ia_name_with_resolution.split(
            "_")[1]), int(ia_name_with_resolution.split("_")[2].replace(".trt", ""))])
        self.__changeConfigValue(
            f"{mod_name}_MODEL_PATH", f"yolo/{ia_name_with_resolution}.trt", True)
        self.__changeConfigValue(
            f"{mod_name}_CLASSES_FILE", f"yolo/{ia_name}.names", True)
        self.__changeConfigValue(f"{mod_name}_INPUT_SIZE", resolution)

    # Define the setting page
    def generate_html(self):

        checkbox_demo_mode = Checkbox("demo_mode", "Activate demo mode", lambda new_value: self.__changeConfigValue(
            "ALLOW_DEMO_PAUSES", new_value))
        checkbox_demo_mode.set_checked(self.__config.ALLOW_DEMO_PAUSES)

        # Navigation category
        category_nav = Category("nav", "Navigation:")

        radio_btn_zig_zag = RadioButton("zig_zag", "Zig zag", lambda new_value: self.__changeConfigValue(
            "FORWARD_BACKWARD_PATH", new_value))
        radio_btn_zig_zag.set_checked(self.__config.FORWARD_BACKWARD_PATH)

        radio_btn_spiral = RadioButton(
            "spiral", "Spiral", lambda new_value: self.__changeConfigValue("BEZIER_PATH", new_value))
        radio_btn_spiral.set_checked(self.__config.BEZIER_PATH)

        slider_sides_interval = Slider(
            "sides_interval",
            "Sides interval",
            lambda new_value: self.__changeConfigValue(
                "SPIRAL_SIDES_INTERVAL",
                new_value
            )
        )
        slider_sides_interval.set_number_parameters(
            0, 3000, 5, self.__config.SPIRAL_SIDES_INTERVAL)

        radio_btn_group_path_choice = RadioButtonGroup(
            "path", "Choice of path:")
        radio_btn_group_path_choice.set_column_number(2)
        radio_btn_group_path_choice.set_radio_button_list(
            [radio_btn_zig_zag, radio_btn_spiral])

        checkbox_bad_gps = Checkbox("bad_gps", "Stop if bad GPS", lambda new_value: self.__changeConfigValue(
            "ALLOW_GPS_BAD_QUALITY_STOP", new_value))
        checkbox_bad_gps.set_checked(self.__config.ALLOW_GPS_BAD_QUALITY_STOP)

        checkbox_leaving_protection = Checkbox("leaving_protection", "Stop if out of zone",
                                               lambda new_value: self.__changeConfigValue("ALLOW_FIELD_LEAVING_PROTECTION", new_value))
        checkbox_leaving_protection.set_checked(
            self.__config.ALLOW_FIELD_LEAVING_PROTECTION)

        slider_leaving_protection_distance = Slider("leaving_protection_distance", "Max distance before stop",
                                                    lambda new_value: self.__changeConfigValue("LEAVING_PROTECTION_DISTANCE_MAX", new_value))
        slider_leaving_protection_distance.set_number_parameters(
            1000, 10000, 500, self.__config.LEAVING_PROTECTION_DISTANCE_MAX)

        checkbox_instruction_path = Checkbox("instruction_path", "Display instruction path",
                                             lambda new_value: self.__changeConfigValue("DISPLAY_INSTRUCTION_PATH", new_value))
        checkbox_instruction_path.set_checked(
            self.__config.DISPLAY_INSTRUCTION_PATH)

        category_nav.add_items([radio_btn_group_path_choice, slider_sides_interval, checkbox_bad_gps,
                               checkbox_leaving_protection, slider_leaving_protection_distance, checkbox_instruction_path])

        # Detection category
        category_detection = Category("detec", "Detection:")

        selector_periph = Selector(
            "periph", "Periphery mod:", lambda new_value: self.__set_ia_in_config("PERIPHERY", new_value))
        selector_periph.set_content_list(self.__get_ia_list(
            "../yolo"), self.__config.PERIPHERY_MODEL_PATH.split("yolo/")[1].split(".trt")[0])
        selector_periph.set_choose_description(
            "Please choose artificial intelligence")

        slider_periph = Slider("periph", "Threshold", lambda new_value: self.__changeConfigValue(
            "PERIPHERY_CONFIDENCE_THRESHOLD", new_value))
        slider_periph.set_number_parameters(
            0.1, 0.8, 0.1, self.__config.PERIPHERY_CONFIDENCE_THRESHOLD)

        selector_precise = Selector(
            "precise", "Precise mod:", lambda new_value: self.__set_ia_in_config("PRECISE", new_value))
        selector_precise.set_content_list(self.__get_ia_list(
            "../yolo"), self.__config.PRECISE_MODEL_PATH.split("yolo/")[1].split(".trt")[0])
        selector_precise.set_choose_description(
            "Please choose artificial intelligence")

        slider_precise = Slider("precise", "Threshold", lambda new_value: self.__changeConfigValue(
            "PRECISE_CONFIDENCE_THRESHOLD", new_value))
        slider_precise.set_number_parameters(
            0.1, 0.8, 0.1, self.__config.PRECISE_CONFIDENCE_THRESHOLD)

        # ,radio_btn_group_shooting
        category_detection.add_items(
            [selector_periph, slider_periph, selector_precise, slider_precise])

        # Weeding technique category
        category_weed_removal = Category(
            "weeding_technique", "Weeding technique parameter:")

        slider_extraction_z = Slider("extraction_z", "Extraction depth",
                                     lambda new_value: self.__changeConfigValue("EXTRACTION_Z", new_value))
        slider_extraction_z.set_number_parameters(
            10, 50, 1, self.__config.EXTRACTION_Z)

        slider_cycle = Slider("cycle", "Cycle", lambda new_value: self.__changeConfigValue(
            "EXTRACTIONS_FULL_CYCLES", new_value))
        slider_cycle.set_number_parameters(
            1, 5, 1, self.__config.EXTRACTIONS_FULL_CYCLES)

        # radio_btn_group_weeding_technique
        category_weed_removal.add_items([slider_extraction_z, slider_cycle])

        # Seeding technique category
        category_weed_seeding = Category(
            "seeding_technique", "Seeding parameter:")

        slider_seeder_quantity = Slider("seeder_quantity", "Number of seeding doses",
                                        lambda new_value: self.__changeConfigValue("SEEDER_QUANTITY", new_value))
        slider_seeder_quantity.set_number_parameters(
            0, 2, 1, self.__config.SEEDER_QUANTITY)

        slider_seeder_ext_offset_y = Slider("seeder_ext_offset_y", "Seeder y offset",
                                            lambda new_value: self.__changeConfigValue("SEEDER_EXT_OFFSET_Y", new_value))
        slider_seeder_ext_offset_y.set_number_parameters(
            0, 70, 5, self.__config.SEEDER_EXT_OFFSET_Y)

        category_weed_seeding.add_items(
            [slider_seeder_quantity, slider_seeder_ext_offset_y])

        # Other category
        category_other = Category("other", "Other")

        selector_language = Selector("language", "Language:", lambda new_value: self.__changeConfigValue(
            "UI_LANGUAGE", new_value, is_str=True))
        selector_language.set_content_list(
            self.__ui_languages["Supported Language"], self.__config.UI_LANGUAGE)
        selector_language.set_choose_description("Please choose language")

        btn_calibrate = Button("calibrate", "Calibrate")
        btn_calibrate.set_on_click_fct_name("go_to_page('calibrate')")
        btn_calibrate.set_color(Button.WHITE)

        btn_screening = Button("actuator_screening", "Actuator screening")
        btn_screening.set_on_click_fct_name("go_to_page('actuator_screening')")
        btn_screening.set_color(Button.WHITE)

        btn_restart_app = Button("reboot_app", "Restart application")
        btn_restart_app.set_on_click_fct_name("go_to_page('restart_ui', true)")
        btn_restart_app.set_color(Button.WHITE)

        btn_restart_robot = Button("reboot_robot", "Restart robot")
        btn_restart_robot.set_on_click_fct_name("go_to_page('reboot', true)")
        btn_restart_robot.set_color(Button.WHITE)

        btn_group_restart = ButtonGroup("btn_restart", "Restart button", [
                                        btn_restart_app, btn_restart_robot])

        btn_group_tools = ButtonGroup("btn_tools", "Tools button", [
                                        btn_calibrate, btn_screening])

        category_other.add_items([selector_language, btn_group_tools, btn_group_restart])

        # Setting page

        self.__setting_generator.add_items(
            [checkbox_demo_mode, category_nav, category_detection, category_weed_removal, category_weed_seeding, category_other])

        return self.__setting_generator.generate_html(self.__ui_languages, self.__config.UI_LANGUAGE)
