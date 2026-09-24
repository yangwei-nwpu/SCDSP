library(ggplot2)
library(reshape2)
library(latex2exp)
library(ggsci)
library(gridExtra)
library(grid)

windowsFonts(Times=windowsFont("Times New Roman"))

get_legend <- function(myplot) {
  tmp <- ggplot_gtable(ggplot_build(myplot))
  leg <- which(sapply(tmp$grobs, function(x) x$name) == "guide-box")
  tmp$grobs[[leg]]
}

make_plot <- function(file, title_text,
                      show_legend = FALSE,
                      show_y_text = TRUE,
                      show_y_ticks = TRUE) {
  dat <- read.csv(file)
  colnames(dat) <- gsub("X", " ", colnames(dat))
  dat <- melt(dat, id = "Gap")
  colnames(dat) <- c("Gap", "CL", "count")
  names(dat)[names(dat) == "Gap"] <- "Dev"
  
  p <- ggplot(dat, aes(x = Dev, y = count, group = CL, color = CL, shape = CL)) +
    geom_point(size = 2.8) +
    geom_line(linetype = 2) +
    labs(
      x = TeX("$\\textit{Dev}\\%$"),
      y = if (show_y_text) "Counts" else NULL,
      title = title_text
    ) +
    scale_x_continuous(breaks = seq(1, 10, 1)) +
    scale_y_continuous(
      limits = c(0, 24),
      breaks = seq(0, 24, 4),
      labels = if (show_y_text) seq(0, 24, 4) else rep("", 7)
    ) +
    scale_color_npg() +
    theme_bw(base_family = "Times New Roman") +
    theme(
      panel.grid.major = element_blank(),
      panel.grid.minor = element_blank(),
      panel.background = element_rect(fill = "transparent", colour = NA),
      plot.background = element_rect(fill = "transparent", colour = NA),
      
      plot.title = element_text(
        hjust = 0.5, size = 14, family = "Times",face = "bold"
      ),
      axis.title.x = element_text(
        size = 12, family = "Times",face = "bold"
      ),
      axis.text.x  = element_text(
        size = 12, family = "Times"
      ),
      axis.title.y = if (show_y_text)
        element_text(size = 12, family = "Times", face = "bold")
      else element_blank(),
      axis.text.y  = if (show_y_text)
        element_text(size = 12, family = "Times")
      else element_text(colour = NA, size = 12, family = "Times"),
      
      axis.ticks.length.x = unit(-0.12, "cm"),
      axis.ticks.length.y = unit(-0.12, "cm"),
      axis.ticks.y = if (show_y_ticks) element_line() else element_blank(),
      
      legend.position = if (show_legend) "bottom" else "none",
      legend.title = element_blank(),
      legend.text = element_text(
        size = 12, family = "Times"
      ),
      legend.background = element_blank(),
      
      plot.margin = margin(8, 8, 2, 8)
    )
  
  return(p)
}

p_legend <- make_plot(
  "PerfGuaA.csv", "SetA",
  show_legend = TRUE,
  show_y_text = TRUE,
  show_y_ticks = TRUE
)
legend_grob <- get_legend(p_legend)

pA <- make_plot(
  "PerfGuaA.csv", "SetA",
  show_legend = FALSE,
  show_y_text = TRUE,
  show_y_ticks = TRUE
)

pB <- make_plot(
  "PerfGuaB.csv", "SetB",
  show_legend = FALSE,
  show_y_text = FALSE,
  show_y_ticks = TRUE
)

pC <- make_plot(
  "PerfGuaC.csv", "SetC",
  show_legend = FALSE,
  show_y_text = FALSE,
  show_y_ticks = TRUE
)


top_row <- arrangeGrob(
  pA, pB, pC,
  ncol = 3,
  widths = c(1.08, 1, 1)
)


final_plot <- arrangeGrob(
  top_row,
  legend_grob,
  ncol = 1,
  heights = c(10, 0.8)
)

grid.newpage()
grid.draw(final_plot)

pdf("PerfGua_ABC_final_TNR.pdf", width = 12, height = 4.2)
grid.newpage()
grid.draw(final_plot)
dev.off()
