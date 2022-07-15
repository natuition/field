from unicodedata import category


class ItemInterface:

    def __init__(self, id: str, label: str):
        self._id: str = id
        self._label: str = label

    def generate_html(self)-> str :
        raise Exception("NotImplementedException")

class RadioButton(ItemInterface):

    def __init__(self, id: str, label: str):
        super().__init__(id, label)

    def generate_html(self)-> str :
        return  f"""<div class="checkbox" style="margin-top: 0px !important;">
                        <label class="checkbox__item">
                            <p class="checkbox__item--title unselectable">
                                {self._label}
                            </p>
                            <input type="checkbox" id="{self.__class__.__name__}_{self._id}">
                            <span class="checkmark">
                                <svg xmlns="http://www.w3.org/2000/svg" width="22" height="22" fill="#47a857" class="bi bi-check" viewBox="0 0 16 16">
                                    <circle cx="8.5" cy="8" r="4" fill="#47a857" />
                                </svg>
                            </span>
                        </label>
                    </div>"""

class Checkbox(ItemInterface):

    def __init__(self, id: str, label: str):
        super().__init__(id, label)

    def generate_html(self)-> str:
        return  f"""<div class="checkbox">
                        <label class="checkbox__item">
                            <p class="checkbox__item--title unselectable">
                                {self._label}
                            </p>
                            <input type="checkbox" id="{self.__class__.__name__}_{self._id}">
                            <span class="checkmark">
                                <svg xmlns="http://www.w3.org/2000/svg" width="22" height="22" fill="#47a857" class="bi bi-check" viewBox="0 0 16 16">
                                    <path d="M10.97 4.97a.75.75 0 0 1 1.07 1.05l-3.99 4.99a.75.75 0 0 1-1.08.02L4.324 8.384a.75.75 0 1 1 1.06-1.06l2.094 2.093 3.473-4.425a.267.267 0 0 1 .02-.022z"/>
                                </svg>
                            </span>
                        </label>
                    </div>"""

class Selector(ItemInterface):

    def __init__(self, id: str, label: str, content_list: list, default_content_index: int, choose_description: str):
        super().__init__(id, label)
        self._content_list: list = content_list
        self._default_content_index: int = default_content_index
        self._choose_description: str = choose_description

    def generate_html(self)-> str :
        content_list_html = [f"<option value='{content}'>{content}</option>" for content in self._content_list]
        content_list_html = "\n".join(content_list_html)
        return  f"""<div class="checklist__select">
                        <div class="checklist__select--title">
                            {self._label}
                        </div>
                        <div class="checklist__select--role">
                            <select id="{self.__class__.__name__}_{self._id}">
                                <option value='{self._choose_description}' disabled>{self._choose_description}</option>
                                {content_list_html}
                            </select>
                            <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="#9ca2b4" class="bi bi-caret-down-fill position-absolute" viewBox="0 0 16 16">
                                <path d="M7.247 11.14L2.451 5.658C1.885 5.013 2.345 4 3.204 4h9.592a1 1 0 0 1 .753 1.659l-4.796 5.48a1 1 0 0 1-1.506 0z"></path>
                            </svg>
                        </div>
                    </div>"""

class Slider(ItemInterface):

    def __init__(self, id: str, label: str, min: float, max: float, step: float, default: float):
        super().__init__(id, label)
        self._min: float = min
        self._max: float = max
        self._step: float = step
        self._default: float = default

    def generate_html(self)-> str :
        return  f"""<div class="ruler__main--assets">
                        <div class="ruler__assets--title">
                            <p>{self._label}</p>
                            <span style="text-align: center;" class="no_format" id="{self.__class__.__name__}_{self._id}_span">{self._default}</span>
                        </div>
                        <div class="ruler__assets--line">
                            <input type="range" min="{self._min}" max="{self._max}" value="{self._default}" step="{self._step}" id="{self.__class__.__name__}_{self._id}_range" oninput="range('{self.__class__.__name__}_{self._id}_range', '{self.__class__.__name__}_{self._id}_span')"">
                        </div>
                    </div>
                    <script defer>
                        range("{self.__class__.__name__}_{self._id}_range", "{self.__class__.__name__}_{self._id}_span")
                    </script>"""

class Button(ItemInterface):

    GREEN = ["71", "168", "87"]
    RED = ["234", "40", "40"]

    def __init__(self, id: str, label: str, color_rgb: 'list[int]', on_click_ftc_name: str):
        super().__init__(id, label)
        self._color_rgb: list[int] = color_rgb
        self._on_click_ftc_name: str = on_click_ftc_name

    def generate_html(self)-> str :
        return f""" <button name="{self.__class__.__name__}_{self._id}" id="{self.__class__.__name__}_{self._id}" class="setting_btn unselectable" onclick="{self._on_click_ftc_name}" style="background-color: rgb({", ".join(self._color_rgb)})!important; box-shadow: 0px 4px 14px 0px rgba({", ".join(self._color_rgb)}, 0.6)!important;">
                        <span class="submit">{self._label}</span>
                        <span class="loading"><i class="fas fa-sync-alt fa-spin"></i></span>
                        <span class="check"><i class="fas fa-check scale"></i></span>
                    </button>"""

class RadioButtonGroup(ItemInterface):

    def __init__(self, id: str, label: str, column_number: int, radio_button_list: 'list[RadioButton]'):
        super().__init__(id, label)
        self._column_number: int = column_number
        self._radio_button_list: list[RadioButton] = radio_button_list

    def generate_html(self)-> str :
        radio_button_html = ""
        for radio_btn in self._radio_button_list:
            radio_button_html += radio_btn.generate_html()+"\n"
        return f""" <div class="checklist__select--title" style="margin-bottom: 0px !important;">
                        {self._label}
                    </div>
                    <div class="radio_ckeckbox" style="grid-template-columns:{(str(100/self._column_number)+"% ")*self._column_number};">
                        {radio_button_html}
                    </div>"""

class ButtonGroup(ItemInterface):

    def __init__(self, id: str, label: str, button_list: 'list[Button]'):
        super().__init__(id, label)
        self._button_list: list[Button] = button_list

    def generate_html(self)-> str :
        button_html = ""
        for btn in self._button_list:
            button_html += btn.generate_html()+"\n"
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

    def __get_string_items(self)-> str:
        str_items = ""
        for item  in self.__items_list:
            str_items += item.generate_html() + "\n"
        return str_items

    def generate_html(self)-> str :
        return f""" <section class="setting__category">
                        <h1 class="setting__category--title unselectable">
                            {self._label}
                        </h1>
                        {self.__get_string_items()}
                    </section>"""

class SettingGenerator():

    def __init__(self):
        self.__items_list: list[ItemInterface] = list()
        self.__btn_save = Button("save", "Save",Button.GREEN,"save_fct")
        self.__btn_cancel = Button("cancel", "Cancel",Button.RED,"cancel_fct")
        self.__btn_group_save_cancel = ButtonGroup("btn_restart", "Restart button", [self.__btn_save,self.__btn_cancel])

    def add_item(self, item: ItemInterface):
        self.__items_list.append(item)

    def add_items(self, items: 'list[ItemInterface]'):
        self.__items_list.extend(items)

    def __get_string_items(self)-> str:
        str_items = ""
        for item  in self.__items_list:
            str_items += item.generate_html() + "\n"
        return str_items + self.__btn_group_save_cancel.generate_html()

    def generate_html(self)-> str :
        return f""" <section class="setting">
                        {self.__get_string_items()}
                    </section>"""

def setting_page_generate_html():
    
    category_nav = Category("nav", "Navigation:")
    radio_btn_zig_zag = RadioButton("zig_zag", "Zig zag")
    radio_btn_spiral = RadioButton("spiral", "Spiral")
    radio_btn_group_path_choice = RadioButtonGroup("path", "Choice of path:",2,[radio_btn_zig_zag,radio_btn_spiral])
    checkbox_bad_gps = Checkbox("bad_gps", "Stop if bad GPS")

    category_nav.add_items([radio_btn_group_path_choice,checkbox_bad_gps])

    category_detection = Category("detec", "Detection:")
    selector_periph = Selector("periph", "Periphery mod:", ["Y16","Y17"],0,"Please choose artificial intelligence")
    slider_periph = Slider("periph", "Threshold",0.1,0.8,0.1,0.2)
    selector_precise = Selector("precise", "Precise mod:", ["Y16","Y17"],0,"Please choose artificial intelligence")
    slider_precise = Slider("precise", "Threshold",0.1,0.8,0.1,0.2)
    radio_btn_mono = RadioButton("mono", "Mono")
    radio_btn_stereo = RadioButton("stereo", "Stereo")
    radio_btn_group_shooting = RadioButtonGroup("shooting", "Choice of picture shoot:",2,[radio_btn_mono,radio_btn_stereo])

    category_detection.add_items([selector_periph,slider_periph,selector_precise,slider_precise,radio_btn_group_shooting])

    category_weed_removal = Category("weeding_technique", "Weeding technique parameter:")
    selector_weed_removal_technique = Selector("weeding_technique", "Weeding techniques:", ["Drilling","Milling"],0,"Please choose weeding technique")
    slider_cycle = Slider("cycle", "Cycle",1,5,1,2)

    category_weed_removal.add_items([selector_weed_removal_technique,slider_cycle])

    category_other = Category("other", "Other")
    selector_language = Selector("language", "Language:", ["fr","en","nl"],0,"Please choose language")
    btn_restart_app = Button("reboot_app", "Restart application",Button.GREEN,"reboot_app_fct")
    btn_restart_robot = Button("reboot_robot", "Restart robot",Button.GREEN,"reboot_robot_fct")
    btn_group_restart = ButtonGroup("btn_restart", "Restart button", [btn_restart_app,btn_restart_robot])

    category_other.add_items([selector_language,btn_group_restart])

    setting_generator = SettingGenerator()
    setting_generator.add_items([category_nav,category_detection,category_weed_removal,category_other])

    return setting_generator.generate_html()

if __name__ == "__main__":
    print(setting_page_generate_html())