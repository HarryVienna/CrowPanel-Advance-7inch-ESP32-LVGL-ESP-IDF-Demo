SET(SOURCES screens/ui_ScreenTest.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    fonts/ui_font_Digital48.c)

add_library(ui ${SOURCES})
