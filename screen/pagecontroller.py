import customtkinter as ctk
from screen.pages import EmergencyPage, StatsPage, HomePage, DevModePage
# Global appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Font constants
FONT_TITLE = ("Arial", 48)
FONT_BUTTON = ("Arial", 36)
FONT_PIN_ENTRY = ("Arial", 48)

class PageController(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.pages = {}
        self.current_page = None
        self.page_history = []
        self.create_pages()

    def create_pages(self):
        # Get the root app instance
        app = self.master

        self.pages = {
            "Emergency": EmergencyPage(self, app=app),
            "Home": HomePage(self, app=app),
            "Stats": StatsPage(self, app=app),
            "Dev Mode": DevModePage(self, app=app),
        }

        for page in self.pages.values():
            page.place(relx=0, rely=0, relwidth=1, relheight=1)

        self.show_page("Home", remember=False)

    def show_page(self, page_name, remember=True):
        if page_name not in self.pages:
            return

        if self.current_page and remember:
            self.page_history.append(self.current_page)
        if self.current_page:
            self.current_page.lower()

        self.current_page = self.pages[page_name]
        self.current_page.lift()

    def go_back(self):
        if self.page_history:
            previous_page = self.page_history.pop()
            if isinstance(previous_page, DevModePage):
                previous_page.reset()

            if self.current_page:
                self.current_page.lower()
            self.current_page = previous_page
            self.current_page.lift()
        else:
            print("No page history to go back to!")
