from unicodedata import category
from typing import Callable
from flask_socketio import SocketIO
import importlib.util
import sys

class ItemInterface:

    def __init__(self, id: str, label: str, modif_callback: Callable = None):
        self._id: str = id
        self._label: str = label
        self._modif_callback: str = modif_callback
        self._html_id = f"{self.__class__.__name__}-{self._id}"

    def save_modif(self, new_value):
        self._modif_callback(self, new_value)

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        raise Exception("NotImplementedException")

class RadioButton(ItemInterface):

    def __init__(self, id: str, label: str, modif_callback: Callable):
        super().__init__(id, label, modif_callback)
        self.__checked: str = ""

    def set_checked(self, checked: bool):
        if checked:
            self.__checked = "checked"
        else:
            self.__checked = ""

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        return  f"""<div class="checkbox" style="margin-top: 0px !important;">
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

    def __init__(self, id: str, label: str, modif_callback: Callable):
        super().__init__(id, label, modif_callback)
        self.__checked: str = ""

    def set_checked(self, checked: bool):
        if checked:
            self.__checked = "checked"
        else:
            self.__checked = ""

    def generate_html(self, ui_languages: dict, ui_language: str)-> str:
        return  f"""<div class="checkbox">
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

    def __init__(self, id: str, label: str, modif_callback: Callable):
        super().__init__(id, label, modif_callback)
        self._content_list: list = None
        self._default_content_index: int = None
        self._choose_description: str = None

    def set_content_list(self, content_list: list, default_content_index: int):
        self._content_list: list = content_list
        self._default_content_index: int = default_content_index
    
    def set_choose_description(self, choose_description: str):
        self._choose_description: str = choose_description

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        content_list_html = [f"<option value='{content}'"+(f" selected='selected'>{content}</option>" if count== self._default_content_index else f">{content}</option>") for count, content in enumerate(self._content_list)]
        content_list_html = "\n".join(content_list_html)
        return  f"""<div class="checklist__select">
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

    def __init__(self, id: str, label: str, modif_callback: Callable):
        super().__init__(id, label, modif_callback)
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

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        return  f"""<div class="ruler__main--assets">
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

    def __init__(self, id: str, label: str, modif_callback: Callable):
        super().__init__(id, label, modif_callback)
        self._color_rgb: list[int] = None
        self._on_click_fct_name = ""

    def set_color(self, color_rgb: 'list[int]'):
        self._color_rgb: list[int] = color_rgb

    def set_on_click_fct_name(self, fct_name):
        self._on_click_fct_name = fct_name

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        return f""" <button name="{self._html_id}" id="{self._html_id}" onclick="{self._on_click_fct_name}" class="setting_btn unselectable" style="background-color: rgb({", ".join(self._color_rgb)})!important; box-shadow: 0px 4px 14px 0px rgba({", ".join(self._color_rgb)}, 0.6)!important;">
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

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        radio_button_html = ""
        for radio_btn in self._radio_button_list:
            radio_button_html += radio_btn.generate_html(ui_languages, ui_language)+"\n"
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

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
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

    def __get_string_items(self, ui_languages: dict, ui_language: str)-> str:
        str_items = ""
        for item  in self.__items_list:
            str_items += item.generate_html(ui_languages, ui_language) + "\n"
        return str_items

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        return f""" <section class="setting__category">
                        <h1 class="setting__category--title unselectable">
                            {ui_languages[self._label][ui_language]}
                        </h1>
                        {self.__get_string_items(ui_languages, ui_language)}
                    </section>"""

class SettingGenerator():

    def __init__(self):
        self.__items_list: list[ItemInterface] = list()

        self.__btn_save = Button("save", "Save", callable)
        self.__btn_save.set_color(Button.GREEN)
        self.__btn_save.set_on_click_fct_name("save_setting()")

        self.__btn_cancel = Button("cancel", "Cancel", callable)
        self.__btn_cancel.set_color(Button.RED)

        self.__btn_group_save_cancel = ButtonGroup("btn_save_cancel", "Save Cancel button", [self.__btn_save,self.__btn_cancel])

    def add_item(self, item: ItemInterface):
        self.__items_list.append(item)

    def add_items(self, items: 'list[ItemInterface]'):
        self.__items_list.extend(items)

    def __get_string_items(self, ui_languages: dict, ui_language: str)-> str:
        str_items = ""
        for item  in self.__items_list:
            str_items += item.generate_html(ui_languages, ui_language) + "\n"
        return str_items + self.__btn_group_save_cancel.generate_html(ui_languages, ui_language)

    def generate_html(self, ui_languages: dict, ui_language: str)-> str :
        return f""" <section class="setting">
                        {self.__get_string_items(ui_languages, ui_language)}
                    </section>"""

def callable(object: ItemInterface, new_value):
    print(f"{object.__class__.__name__}_{object._id} : {new_value}")

class SettingPageManager:

    def __init__(self, socket_io: SocketIO, ui_languages: dict, ui_language: str):
        self.__socket_io: SocketIO = socket_io
        self.__ui_languages = ui_languages
        self.__ui_language = ui_language
        self.__socket_io.on_event('data', self.__save_event, namespace='/save_setting')
        self.__reload_config()

    def __save_event(self, save_data):
        print(save_data)

    def __reload_config(self):
        spec = importlib.util.spec_from_file_location("config.name", "../config/config.py")
        self.__config = importlib.util.module_from_spec(spec)
        sys.modules["config.name"] = self.__config
        spec.loader.exec_module(self.__config)

    def generate_html(self):

        #Navigation category  
        category_nav = Category("nav", "Navigation:")

        radio_btn_zig_zag = RadioButton("zig_zag", "Zig zag", callable)
        radio_btn_zig_zag.set_checked(self.__config.FORWARD_BACKWARD_PATH)

        radio_btn_spiral = RadioButton("spiral", "Spiral", callable)
        radio_btn_spiral.set_checked(self.__config.BEZIER_PATH)

        slider_sides_interval = Slider("sides_interval", "Sides interval:", callable)
        slider_sides_interval.set_number_parameters(0,3000,5,10)

        radio_btn_group_path_choice = RadioButtonGroup("path", "Choice of path:")
        radio_btn_group_path_choice.set_column_number(2)
        radio_btn_group_path_choice.set_radio_button_list([radio_btn_zig_zag,radio_btn_spiral])

        checkbox_bad_gps = Checkbox("bad_gps", "Stop if bad GPS", callable)
        checkbox_bad_gps.set_checked(self.__config.GPS_QUALITY_IGNORE)

        category_nav.add_items([radio_btn_group_path_choice,slider_sides_interval,checkbox_bad_gps])

        #Detection category
        category_detection = Category("detec", "Detection:")

        selector_periph = Selector("periph", "Periphery mod:", callable)
        selector_periph.set_content_list(["Y16","Y17"],0)
        selector_periph.set_choose_description("Please choose artificial intelligence")

        slider_periph = Slider("periph", "Threshold", callable)
        slider_periph.set_number_parameters(0.1,0.8,0.1,0.2)

        selector_precise = Selector("precise", "Precise mod:", callable)
        selector_precise.set_content_list(["Y16","Y17"],0)
        selector_precise.set_choose_description("Please choose artificial intelligence")

        slider_precise = Slider("precise", "Threshold", callable)
        slider_precise.set_number_parameters(0.1,0.8,0.1,0.2)

        radio_btn_mono = RadioButton("mono", "Mono", callable)
        radio_btn_mono.set_checked(len(self.__config.CAMERA_POSITIONS)==1)

        radio_btn_stereo = RadioButton("stereo", "Stereo", callable)
        radio_btn_stereo.set_checked(len(self.__config.CAMERA_POSITIONS)>1)

        radio_btn_group_shooting = RadioButtonGroup("shooting", "Choice of picture shoot:")
        radio_btn_group_shooting.set_column_number(2)
        radio_btn_group_shooting.set_radio_button_list([radio_btn_mono,radio_btn_stereo])

        category_detection.add_items([selector_periph,slider_periph,selector_precise,slider_precise,radio_btn_group_shooting])

        #Weeding technique category
        category_weed_removal = Category("weeding_technique", "Weeding technique parameter:")

        radio_btn_drilling = RadioButton("drilling", "Drilling", callable)
        radio_btn_drilling.set_checked(self.__config.EXTRACTION_MODE==1)

        radio_btn_milling = RadioButton("milling", "Milling", callable)
        radio_btn_milling.set_checked(self.__config.EXTRACTION_MODE==2)

        radio_btn_group_weeding_technique = RadioButtonGroup("shooting", "Choice of weeding technique:")
        radio_btn_group_weeding_technique.set_column_number(2)
        radio_btn_group_weeding_technique.set_radio_button_list([radio_btn_drilling,radio_btn_milling])

        slider_cycle = Slider("cycle", "Cycle", callable)
        slider_cycle.set_number_parameters(1,5,1,2)

        category_weed_removal.add_items([radio_btn_group_weeding_technique,slider_cycle])

        #Other category
        category_other = Category("other", "Other")

        selector_language = Selector("language", "Language:", callable)
        selector_language.set_content_list(self.__ui_languages["Supported Language"],self.__ui_languages["Supported Language"].index(self.__ui_language))
        selector_language.set_choose_description("Please choose language")

        btn_restart_app = Button("reboot_app", "Restart application", callable)
        btn_restart_app.set_color(Button.GREEN)

        btn_restart_robot = Button("reboot_robot", "Restart robot", callable)
        btn_restart_robot.set_color(Button.GREEN)

        btn_group_restart = ButtonGroup("btn_restart", "Restart button", [btn_restart_app,btn_restart_robot])

        category_other.add_items([selector_language,btn_group_restart])

        #Setting page
        setting_generator = SettingGenerator()
        setting_generator.add_items([category_nav,category_detection,category_weed_removal,category_other])

        return setting_generator.generate_html(self.__ui_languages, self.__ui_language)