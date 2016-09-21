def set_file_path( verbose=False ):  
    '''
    execute the set_file_path function of the super package. 
    -> an absolute path has to be set only at root level of packages.
    '''
    import os 
    import sys
    
    file_path   = os.path.abspath( os.path.dirname( __file__ ) )  
    super_path  = os.path.abspath( os.path.join( file_path, '../..' ))
    
    if verbose :
        print 
        print file_path
        print super_path
        print 
    
    sys.path.append(  super_path )
    
    return True

'''
# Header
# Filename:      dashboard.R
# Description:   This project, aims to create a ref class containing multiple objects
#                that can issue a shiny dashboard 
# Author:        Nima Ramezani Taghiabadi
# Email :        nima.ramezani@cba.com.au
# Start Date:    22 January 2016
# Last Revision: 08 September 2016
# Version:       1.6.2

# Version History:

# Version   Date                Action
# ---------------------------------------
# 1.6.0     04 July 2016        A number of changes made. Refer to dashboard.R (version 1.6)

# Changes from version 1.5:
# 1- method io.str() transferred from dash.tools.R
# 2- Starting letters of input and cloth types changed to lowercase to match shiny namespace: Example: RadioButton changed to radioButton
# 3- Valid Container types separated from valid input types
# 4- class properties 'inputs' and 'outputs' consolidated to one single list called 'items' which must be a named list
# 5- layouts must be specified by item names not numbers
# 6- field ID removed and replaced by item name

# 1.6.2     08 September 2016   Argument inline added to checkboxGroupInput.

valid.box.statuses = c('primary', # Blue (sometimes dark blue),
                 'success', # Green
                 'info',    # Blue
                 'warning', # Orange
                 'danger'  # Red
                 )
valid.colors = c('red', 'yellow', 'aqua', 'blue', 'light-blue', 'green', 'navy', 'teal', 'olive', 'lime', 'orange', 
                  'fuchsia', 'purple', 'maroon', 'black')

valid.input.types = c("radioButton", "sliderInput", "actionButton", "checkboxInput", "checkboxGroupInput", "selectInput", "dateInput", 
                      "dateRangeInput", "fileInput", "numericInput")

valid.output.types = c("uiOutput", "dynamicInput", "plotOutput", "verbatimTextOutput", "textOutput", "tableOutput", "dataTableOutput", "htmlOutput", 
                       "gglVisChartOutput", "rChartsdPlotOutput", 'dygraphOutput', 'plotlyOutput', 'amChartsOutput', 
                       "leafletOutput", "infoBoxOutput", "valueBoxOutput",  
                       "static")

valid.container.types = c("column", "box", "fluidPage", "dashboardPage", 
                          "sidebarLayout", "navbarPage", "navbarMenu", "tabPanel", "wellPanel")


valid.cloth.types = c("box", "infoBox", "valueBox", "column", "wellPanel")

valid.navbar.positions = c("static-top", "fixed-top", "fixed-bottom")

valid.dashboard.skins  = c("blue", "black", "purple", "green", "red", "yellow")

# Returns the number of elements of the given list which are either 'list' or 'character'
nListOrCharItems = function(lst){
  sum = 0
  for (i in lst){
    if (inherits(i, 'list') | inherits(i, 'character')){sum = sum + 1}
  }
  return(sum)
}


#' @exportClass DASHBOARD
DASHBOARD <- setRefClass("DASHBOARD", 
 fields = list(
   # a list of objects which are synced with dashboard inputs and 
   # provide service for dashboard outputs
   name        = "character",
   items       = "list",
   king.layout = "list",
   objects     = "list",
   observers   = "character",
   reactives   = "character"
 ),
 
 methods = list(
   # Class constructor  
   initialize = function(values = list(), name = "niravis.dashboard", ...){
     callSuper(...)
     # Field assignment:
     name  <<- name
     self.verify()
     # then output[i]$title has priority, then output[i]$cloth$title
   }, 
   
   self.verify = function(){
     verify(items,  'list', varname = 'items')
     for (i in items){
       verify(i, 'list', names_in = c('type'), varname = "items[['" %+% i %+% "']]")
       verify(i$type, 'character', range = c(valid.input.types, valid.container.types, valid.output.types), varname = "items[['" %+% i %+% "']]$type")
     }
   },

   io.str = function(i){
     if (items[[i]]$type %in% valid.output.types){
       scr = paste0("items[['", i ,"']]$object")
     } else if (items[[i]]$type %in% c(valid.input.types, valid.container.types)) {
       scr = paste0("get.item.object('", i,"')")
     } else {return("")}
     return(scr)
   },
   
   io.clothed.str = function(i, cloth = NULL){
     s = io.str(i)
     if (is.null(cloth)){return(s)}
     
     verify(cloth, "list")
     verify(cloth$type, "character", range = valid.cloth.types)
     
     cloth.str = "items[['" %+% i %+% "']]$cloth"
     
     switch(cloth$type, 
            "box"       = {scr = "box("
                     if (!is.null(cloth$title)){
                       verify(cloth$title, 'character')
                       scr = paste0(scr, "title = ", cloth.str, "$title,")
                     }
                     if (!is.null(cloth$footer)){
                       verify(cloth$footer, 'character')
                       scr = paste0(scr, "title = ", cloth.str, "$title,")
                     }
                     if (!is.null(cloth$status)){
                       verify(cloth$status, 'character', range = valid.box.statuses)
                       scr = paste0(scr, "status = ", cloth.str, "$status,")
                     }
                     if (!is.null(cloth$solidHeader)){
                       verify(cloth$solidHeader, 'logical', range = c(T,F))
                       scr = paste0(scr, "solidHeader = ", cloth.str, "$solidHeader,")
                     }
                     if (!is.null(cloth$background)){
                       verify(cloth$background, 'character', range = valid.colors)
                       scr = paste0(scr, "background = ", cloth.str, "$background,")
                     }
                     
                     if (!is.null(cloth$weight)){
                       verify(cloth$weight, c('numeric', 'integer'), range = c(1,12))
                       scr = paste0(scr, "width = ", cloth.str, "$weight,")
                     }
                     
                     if (!is.null(cloth$height)){
                       verify(cloth$height, 'character')
                       scr = paste0(scr, "height = ", cloth.str, "$height,")
                     }
                     
                     if (!is.null(cloth$collapsible)){
                       verify(cloth$collapsible, 'logical', range = c(T,F))
                       scr = paste0(scr, "collapsible = ", cloth.str, "$collapsible,")
                     }  
                     if (!is.null(cloth$collapsed)){
                       verify(cloth$collapsed, 'logical', range = c(T,F))
                       scr = paste0(scr, "collapsed = ", cloth.str, "$collapsed,")
                     }
                     scr = scr %+% s %+% ")"},
            "infoBox"   = {scr = "infoBox("
                         ttl = verify(cloth$title, 'character', default = '')
                         scr = paste0(scr, "title = ", "'", ttl, "', ")
                         
                         #if (!is.null(cloth$title)){
                           #verify(cloth$title, 'character')
                           #scr = paste0(scr, "title = ", cloth.str, "$title,")
                         #} else {}
                         if (!is.null(cloth$subtitle)){
                           verify(cloth$subtitle, 'character')
                           scr = paste0(scr, "subtitle = ", cloth.str, "$subtitle,")
                         }
                         if (!is.null(cloth$icon)){
                           verify(cloth$icon, 'character')
                           scr   = paste0(scr, "icon = shiny::icon(", cloth.str, "$icon),")
                         }
                         if (!is.null(cloth$color)){
                           verify(cloth$color, 'character', range = valid.colors)
                           scr = paste0(scr, "color = ", cloth.str, "$color,")
                         }  
                         if (!is.null(cloth$weight)){
                           verify(cloth$weight, c('numeric', 'integer'), range = c(1,12))
                           scr = paste0(scr, "width = ", cloth.str, "$weight,")
                         }  
                         if (!is.null(cloth$href)){
                           verify(cloth$href, 'character')
                           scr = paste0(scr, "href = ", cloth.str, "$href,")
                         }  
                         if (!is.null(cloth$fill)){
                           verify(cloth$fill, 'logical', range = c(T,F))
                           scr = paste0(scr, "fill = ", cloth.str, "$fill,")
                         }
                         scr = paste0(scr, "value = ", s,")")},
            "valueBox"  = {scr = "valueBox("
                         if (!is.null(cloth$title)){
                           verify(cloth$subtitle, 'character')
                           scr = paste0(scr, "subtitle = ", cloth.str, "$title,")
                         } else {scr = paste0(scr, "subtitle = '',")}
                         if (!is.null(cloth$icon)){
                           verify(cloth$icon, 'character')
                           scr   = paste0(scr, "icon = shiny::icon(", cloth.str, "$icon),")
                         }
                         if (!is.null(cloth$color)){
                           verify(cloth$color, 'character', range = valid.colors)
                           scr = paste0(scr, "color = ", cloth.str, "$color,")
                         }  
                         if (!is.null(cloth$weight)){
                           verify(cloth$weight, c('numeric', 'integer'), range = c(1,12))
                           scr = paste0(scr, "width = ", cloth.str, "$weight,")
                         }  
                         if (!is.null(cloth$href)){
                           verify(cloth$href, 'character')
                           scr = paste0(scr, "href = ", cloth.str, "$href,")
                         }  
                         scr = paste0(scr, "value = ", s,")")},
            "column"    = {
              scr = paste0("column(", list2Script(cloth, fields = c('offset', 'align')),", ")
              if (!is.null(cloth$weight)){
                verify(cloth$weight, c('numeric', 'integer'), range = c(1,12))
                scr = paste0(scr, "width = ", cloth.str, "$weight,")
              }  
              scr = paste0(s, ")")
            },
            "wellPanel" = {scr = paste0("wellPanel(", s, ")")}
     )
     return(scr)
   },
   
   # only row layout is supported for the sidebar
   # lst.side is a vector of numerics
   # lst.main is a vector of numerics
   layscript.sidebar = function(s = '', lst.side, lst.main){
     s = s %+% "sidebarLayout("
     
     N.side = length(lst.side)
     N.main = length(lst.main)
     
     if (N.side > 0){
       s = s %+% "sidebarPanel("
       s = insert.io.strs(s, lst.side)
       s = s %+% "),"
     }
     
     if (N.main > 0){
       s = s %+% "mainPanel("
       s = insert.io.strs(s, lst.main)
       s = s %+% ")"
     }
     s = s %+% ")"
     return (s)
   },
   
   layscript.dashboard = function(s = '', lst.head, lst.side, lst.body, header.title = NULL, header.titleWidth = NULL){
     N.head = length(lst.head)
     N.side = length(lst.side)
     N.body = length(lst.body)
     
     s = s %+% "dashboardHeader("
     if (!is.null(header.title)){
       s = paste0(s, "title = '", header.title, "'")
       if (N.head > 0 | !is.null(header.titleWidth)){s = s %+% ', '}
     }
     
     if (!is.null(header.titleWidth)){
       s = paste0(s, "titleWidth = ", header.titleWidth)
       if (N.head > 0){s = s %+% ', '}
     }

     if (N.head > 0){s = insert.io.strs(s, lst.head)} else if (is.null(header.title) & is.null(header.titleWidth)){s = s %+% "disable = TRUE"}
     s = s %+% "),"

     s = s %+% "dashboardSidebar("
     if (N.side > 0){s = insert.io.strs(s, lst.side)} else {s = s %+% "disable = TRUE"}
     s = s %+% "),"
     
     s = s %+% "dashboardBody("
     s = insert.io.strs(s, lst.body)
     s = s %+% ")"
     return (s)
   },
   
   layscript = function(layout){
     # todo: verify layout is a list of characters
     N.item = length(layout)
     s = ''
     for (i in sequence(N.item)){
       s = s %+% "get.item.object('" %+% layout[[i]] %+% "'"
       if (i < N.item){s = s %+% ','}
     }
     return(s)
   },
   
   layscript.RCPanel = function(s = "", lst, title = '', is.row = T, col.panel = F){
     N.items = nListOrCharItems(lst)
     for (i in sequence(length(lst))){
       its.list = inherits(lst[[i]], 'list')
       its.char = inherits(lst[[i]], 'character')
       if (its.list | its.char){
         if (is.row){s = paste0(s, "fluidRow(")} else {
           if (its.list){
             if (is.null(lst[[i]]$weight)){
               ww = floor(12/N.items)
             } else {ww = lst[[i]]$weight}
             if (is.null(lst[[i]]$offset)){
               ofst = 0
             } else {ofst = lst[[i]]$offset}
           } else if (its.char) {
             if (is.null(items[[lst[[i]]]]$weight)){
               ww = floor(12/N.items)
             } else {
               ww = items[[lst[[i]]]]$weight
             }
             if (is.null(items[[lst[[i]]]]$offset)){
               ofst = 0
             } else {
               ofst = items[[lst[[i]]]]$offset
             }
           }
           
           s = paste0(s, "column(offset = ",as.character(ofst), ", ")
           s = paste0(s, "width  = ",as.character(ww), ", ")
           if (col.panel){s = paste0(s, 'wellPanel(')}
         }
         
         if (its.list){s = layscript.RCPanel(s, lst[[i]], is.row = !is.row, col.panel = col.panel)}
         else if (its.char) {s = insert.io.strs(s, lst[[i]])}
         s = paste0(s, ')')
         if (col.panel & !is.row){s = paste0(s, ')')}
         if (i < N.items){s = paste0(s, ',')}
       }
     }
     return (s)
   },
   
   insert.io.strs = function(s, vct){
     # vct must be a vector of numerics (later, it can be character too)
     M = length(vct)
     for (j in sequence(M)){
       s = s %+% io.clothed.str(vct[j], cloth = items[[vct[j]]]$cloth)
       if (j < M){s = s %+% ','}
     }
     return(s)
   },
   
   get.item.object = function(i){
     if (is.null(items[[i]]$object)){
       if (is.null(items[[i]]$type)){return(NULL)}
       switch(items[[i]]$type,
              "radioButton" = {
                chcs = verify(items[[i]]$choices, 'character', varname = "items[['" %+% i %+% "']]$choices")
                sltd = verify(items[[i]]$selected, 'character', varname = "items[['" %+% i %+% "']]$selected")
                inl  = verify(items[[i]]$inline, 'logical', varname = "items[['" %+% i %+% "']]$inline", default = F)
                assert(length(chcs) > 1, "radioButton input must have at least two choices!")
                if (is.null(names(chcs))){names(chcs) = chcs}
                obj = radioButtons(i, label = items[[i]]$title, choices = chcs, selected = sltd, inline = inl, width = items[[i]]$width)},
              
              "sliderInput" = {
                xl   = verify(items[[i]]$min    , allowed = c("integer", "numeric"), default = 0)
                xh   = verify(items[[i]]$max    , allowed = c("integer", "numeric"), default = 1)
                x    = verify(items[[i]]$value  , allowed = c("integer", "numeric"), range = c(xl, xh), default = 0.5*(xl+xh))
                an   = verify(items[[i]]$animate, allowed = c("list", "logical"), default = F)
                
                obj = sliderInput(i, label = items[[i]]$title, 
                                  min = xl, max = xh, value = x, step = items[[i]]$step, 
                                  sep  = items[[i]]$sep, pre = items[[i]]$pre, post = items[[i]]$post, 
                                  animate = an)},
              
              "actionButton"   = {obj = actionButton(i, label = items[[i]]$title, width = items[[i]]$width, icon = items[[i]]$icon)},
              "checkboxInput"  = {
                vlu = verify(items[[i]]$value, 'logical', varname = "items[['" %+% i %+% "']]$value", default = F)
                obj = checkboxInput(i, label = items[[i]]$title, value = vlu, width = items[[i]]$width)},
              "checkboxGroupInput" = {
                inl  = verify(items[[i]]$inline, 'logical', varname = "items[['" %+% i %+% "']]$inline", default = F)
                obj  = checkboxGroupInput(i, label = items[[i]]$title, choices = items[[i]]$choices, selected = items[[i]]$selected, inline = inl)},
              "selectInput"    = {
                mltpl = verify(items[[i]]$multiple, 'logical', varname = "items[['" %+% i %+% "']]$multiple", default = F)
                slctz = verify(items[[i]]$selectize, 'logical', varname = "items[['" %+% i %+% "']]$selectize", default = T)
                obj   = selectInput(i, label = items[[i]]$title, choices = items[[i]]$choices, 
                                    selected = items[[i]]$selected, multiple = mltpl, selectize = slctz)},
              "dateInput"      = {obj = dateInput(i, label = items[[i]]$title, value = items[[i]]$value, min = items[[i]]$min, max = items[[i]]$max)},
              "dateRangeInput" = {obj = dateRangeInput(i, label = items[[i]]$title, min = items[[i]]$min, max = items[[i]]$max)},
              "fileInput"      = {obj = fileInput(i, label = items[[i]]$title)},
              "numericInput"   = {obj = numericInput(i, label = items[[i]]$title, value = items[[i]]$value, 
                                                     step = items[[i]]$step, min = items[[i]]$min, max = items[[i]]$max)},

              "column" = {
                scr = "column("
                wdth = verify(items[[i]]$weight, 'numeric' , default = 12)
                ofst = verify(items[[i]]$offset, 'numeric', default = 0)
                scr = paste0(scr,'width = ', wdth, ', offset = ',ofst, ',')
                scr = insert.io.strs(scr, items[[i]]$layout)
                scr = scr %+% ")"
                obj = eval(parse(text = scr))},
              
              "wellPanel" = {
                scr = "wellPanel("
                scr = insert.io.strs(scr, items[[i]]$layout)
                scr = scr %+% ")"
                obj = eval(parse(text = scr))},
              
              "box" = {
                scr = "box("
                wdth = verify(items[[i]]$weight, 'numeric' , default = 12, varname    = paste0("items[['",i,"']]$weight"))
                ttle = verify(items[[i]]$title, 'character' , default = '', varname  = paste0("items[['",i,"']]$title"))
                fotr = verify(items[[i]]$footer, 'character' , default = '', varname = paste0("items[['",i,"']]$footer"))
                stus = verify(items[[i]]$status, 'character' , range = valid.box.statuses, varname = paste0("items[['",i,"']]$status"))
                shdr = verify(items[[i]]$solidHeader, 'logical' , default = 'F', range = c(T, F), varname = paste0("items[['",i,"']]$solidHeader"))
                bgrd = verify(items[[i]]$background, 'character' , range = valid.colors, varname = paste0("items[['",i,"']]$background"))
                cpbl = verify(items[[i]]$collapsible, 'logical' , default = 'F', range = c(T, F), varname = paste0("items[['",i,"']]$collapsible"))
                cpsd = verify(items[[i]]$collapsed, 'logical' , default = 'F', range = c(T, F), varname = paste0("items[['",i,"']]$collapsed"))
                
                if(!is.null(stus)){scr = paste0(scr,"status = '", stus, "',")}
                scr = paste0(scr,'width = ', wdth, ',')
                scr = paste0(scr,"title = '", ttle, "',")
                scr = paste0(scr,'footer = ', fotr, ',')
                scr = paste0(scr,'background = ', bgrd, ',')
                if (shdr){scr = paste0(scr,'solidHeader = T,')}
                if (cpbl){scr = paste0(scr,'collapsible = T,')}
                if (cpsd){scr = paste0(scr,'collapsed = T,')}
                
                scr = insert.io.strs(scr, items[[i]]$layout)
                scr = scr %+% ")"
                obj = eval(parse(text = scr))},
              
              "fluidPage" = {
                scr = "fluidPage("
                if (!is.null(items[[i]]$title)){scr = paste0(scr, "titlePanel('", items[[i]]$title, "', windowTitle = '", items[[i]]$wintitle, "'),")}
                cpl = verify(items[[i]]$col.framed, 'logical', varname = paste0("items[['",i,"']]$col.framed"), range = c(T,F), default = F)
                scr = layscript.RCPanel(s = scr, lst = items[[i]]$layout, col.panel = cpl)
                scr = scr %+% ')'
                obj = eval(parse(text = scr))},
              
              "dashboardPage" = {
                scr = "dashboardPage("
                ttl = verify(items[[i]]$title,  'character', varname = paste0("items[['",i,"']]$title"))
                clr = verify(items[[i]]$color, 'character', varname = paste0("items[['",i,"']]$color"), range = valid.dashboard.skins)
                if (!is.null(ttl)){scr = scr %+% "title = '" %+% ttl %+% "', "}
                if (!is.null(clr)){scr = scr %+% "skin  = '" %+% clr %+% "', "}
                scr = layscript.dashboard(s = scr, lst.head = items[[i]]$layout.head, lst.side = items[[i]]$layout.side, lst.body = items[[i]]$layout.body, header.title = items[[i]]$header.title, header.titleWidth = items[[i]]$header.titleWidth)
                scr = scr %+% ')'
                obj = eval(parse(text = scr))},
              
              "sidebarLayout" = {
                scr = "fluidPage("
                if (!is.null(items[[i]]$title)){scr = paste0(scr, "titlePanel('", items[[i]]$title, "', windowTitle = '", items[[i]]$wintitle, "'),")}
                scr = layscript.sidebar(s = scr, lst.side = items[[i]]$layout.side, lst.main = items[[i]]$layout.main)
                scr = scr %+% ')'
                obj = eval(parse(text = scr))},
              
              "navbarPage" = {  # todo: write specific layscript for this type of container so that it creates tabPanels and menus based on a layout of type list 
                scr = "navbarPage("
                if (!is.null(items[[i]]$title)){scr = scr %+% "title = '" %+% verify(items[[i]]$title, 'character')  %+% "', "}
                if (!is.null(items[[i]]$position)){scr = scr %+% "position = '" %+% verify(items[[i]]$position, 'character', range = valid.navbar.positions)  %+% "', "}
                if (!is.null(items[[i]]$header)){scr = scr %+% "header = '" %+% verify(items[[i]]$header, 'character')  %+% "', "}
                if (!is.null(items[[i]]$footer)){scr = scr %+% "footer = '" %+% verify(items[[i]]$footer, 'character')  %+% "', "}
                if (!is.null(items[[i]]$wintitle)){scr = scr %+% "windowTitle = '" %+% verify(items[[i]]$wintitle, 'character')  %+% "', "}
                if (!is.null(items[[i]]$icon)){scr = scr %+% "icon = icon('" %+% verify(items[[i]]$icon, 'character')  %+% ")', "}
                if (!is.null(i)){scr = scr %+% "id = '" %+% verify(i, 'character')  %+% "', "}
                clp = verify(items[[i]]$collapsible, 'logical', range = c(T,F), default = F) 
                fld = verify(items[[i]]$fluid, 'logical', range = c(T,F), default = T) 
                if (clp) {scr = scr %+% "collapsible = TRUE, "}
                if (!fld){scr = scr %+% "fluid = FASLE, "}
                if (!is.null(items[[i]]$theme)){scr = scr %+% "theme = '" %+% verify(items[[i]]$theme, 'character')  %+% "', "}
                scr = insert.io.strs(scr, items[[i]]$layout)
                scr = scr %+% ")"
                obj = eval(parse(text = scr))},

              "navbarMenu" = {
                scr = "navbarMenu("
                if (!is.null(items[[i]]$title)){scr = scr %+% "title = '" %+% verify(items[[i]]$title, 'character')  %+% "', "}
                if (!is.null(items[[i]]$icon)){scr = scr %+% "icon = icon('" %+% verify(items[[i]]$icon, 'character')  %+% ")', "}
                scr = insert.io.strs(scr, items[[i]]$layout)
                scr = scr %+% ")"
                obj = eval(parse(text = scr))},
              
              "tabPanel" = {
                scr = "tabPanel("
                if (!is.null(items[[i]]$title)){scr = scr %+% "title = '" %+% verify(items[[i]]$title, 'character')  %+% "', "}
                if (!is.null(items[[i]]$icon)){scr = scr %+% "icon = icon('" %+% verify(items[[i]]$icon, 'character')  %+% ")', "}
                scr = insert.io.strs(scr, items[[i]]$layout)
                scr = scr %+% ")"
                obj = eval(parse(text = scr))}
              
       )
       return(obj)
     } else {return(items[[i]]$object)}
   },
   
   dashboard.ui = function(){

     for (i in names(items)){
       # Outputs:
       if (items[[i]]$type %in% valid.output.types){
         if (!is.null(items[[i]]$type)){
           switch(items[[i]]$type,
                  "dynamicInput" = {
                    inl = verify(items[[i]]$inline, 'logical' , range = c(T,F), default = F , varname = "items[['" %+% i %+% "']]$inline")
                    items[[i]]$object <<- uiOutput(i, inline = inl)
                  },
                  "uiOutput" = {
                    items[[i]]$object <<- uiOutput(i)
                    if (!is.null(items[[i]]$cloth)  &  !is.null(items[[i]]$title)){items[[i]]$cloth$title <<- items[[i]]$title}
                  },
                  "plotOutput" = {
                    fields = names(items[[i]])
                    if ('click' %in% fields) {clck  = items[[i]]$click} else {clck = NULL}
                    if ('brush' %in% fields) {brsh  = brushOpts(id = items[[i]]$brush)} else {brsh = NULL}
                    if ('height' %in% fields){hght  = items[[i]]$height} else {hght = "400px"}
                    if ('width' %in% fields) {wdth  = items[[i]]$width} else {wdth = "auto"}
                    items[[i]]$object <<- plotOutput(i, height = hght, width = wdth, click = clck ,brush = brsh)},
                  "verbatimTextOutput" = {items[[i]]$object <<- verbatimTextOutput(i)},
                  "textOutput"         = {items[[i]]$object <<- textOutput(i)},
                  "tableOutput"        = {items[[i]]$object <<- tableOutput(i)},
                  "dataTableOutput"    = {items[[i]]$object <<- DT::dataTableOutput(i)},
                  "htmlOutput"         = {items[[i]]$object <<- htmlOutput(i)},
                  "amChartsOutput"     = {items[[i]]$object <<- amChartsOutput(i)},
                  "dygraphOutput"      = {items[[i]]$object <<- dygraphs::dygraphOutput(i)},
                  "gglVisChartOutput"  = {items[[i]]$object <<- htmlOutput(i)},
                  "leafletOutput"      = {items[[i]]$object <<- leafletOutput(i)},
                  "infoBoxOutput"      = {items[[i]]$object <<- infoBoxOutput(i,  width = items[[i]]$width)},
                  "valueBoxOutput"     = {items[[i]]$object <<- valueBoxOutput(i, width = items[[i]]$width)},
                  "plotlyOutput"       = {
                    wdth = verify(items[[i]]$width, 'character' , default = '100%' , varname = "items[['" %+% i %+% "']]$width")
                    hght = verify(items[[i]]$height, 'character', default = '400px', varname = "items[['" %+% i %+% "']]$height")
                    items[[i]]$object <<- plotlyOutput(i, width = wdth, height = hght)
                  },
                  "rChartsdPlotOutput" = {items[[i]]$object <<- showOutput(i, "dimple")})
         }
       }
     }

     for (i in names(items)){
       # Inputs & Containers
       if (items[[i]]$type %in% c(valid.input.types, valid.container.types)){
         items[[i]]$object <<- get.item.object(i)
       }
     }   
     
     scr.text = layscript(layout = king.layout) %+% ")"
       
     ui.obj <- eval(parse(text = scr.text))
     return(ui.obj)
   }, 
   
   dashboard.server = function(){
     srv_func = function(input, output, session) {
       val = reactiveValues()
       for (i in names(objects)){
         val[[i]] <- objects[[i]]
       }
       # pre.run <- "for (i in names(objects)){if (is.null(val[[i]])){val[[i]] <- objects[[i]]}}"
       
       for (i in names(items)){
         if (items[[i]]$type %in% valid.output.types){
           if (items[[i]]$type != 'static'){
             switch(items[[i]]$type,
                    "dynamicInput"       = {script.func = 'renderUI'},
                    "uiOutput"           = {script.func = 'renderText'},
                    "plotOutput"         = {script.func = 'renderPlot'},
                    "verbatimTextOutput" = {script.func = 'renderPrint'},
                    "textOutput"         = {script.func = 'renderText'},
                    "tableOutput"        = {script.func = 'renderTable'},
                    "dataTableOutput"    = {script.func = 'DT::renderDataTable'},
                    "htmlOutput"         = {script.func = 'renderUI'},
                    "dygraphOutput"      = {script.func = 'dygraphs::renderDygraph'},
                    "gglVisChartOutput"  = {script.func = 'renderGvis'},
                    "leafletOutput"      = {script.func = 'renderLeaflet'},
                    "infoBoxOutput"      = {script.func = 'renderInfoBox'},
                    "valueBoxOutput"     = {script.func = 'renderValueBox'},
                    "amChartsOutput"     = {script.func = 'renderAmCharts'},
                    "plotlyOutput"       = {script.func = 'plotly::renderPlotly'},
                    "rChartsdPlotOutput" = {script.func = 'renderChart2'}   
             )
             open  = '({'
             close = '})'
             script = paste0('output$', i, ' <- ', script.func, open, items[[i]]$srv.func, close)
             eval(parse(text = script))
           }
         }
       }
       
       # observeEvents  (input service functions)
       for (i in names(items)){
         if (items[[i]]$type %in% valid.input.types){
           if (!is.null(items[[i]]$srv.func)){
             script = paste0('observeEvent(input$', i, ',{', items[[i]]$srv.func, '})')
             eval(parse(text = script))
           }
         }
       }
         
       # observers
       # for (obs in observers){eval(parse(text = "observe({" %+% pre.run %+% '\n' %+% obs %+% "})"))}
       for (obs in observers){eval(parse(text = "observe({" %+% obs %+% "})"))}
     }
     return(srv_func)
   }
 )  
)

'''
