library(stringr)

file = "../src/curves.csv"

d <- read.csv2(file,sep=",",dec=".", header = TRUE)

originalNames = names(d)

for ( a in originalNames ) {
    if ( a == "x" ) {
        next
    }
    a_smoothed = paste(a,"smoothed",sep=".")
    idx <- which(is.na(d[,a]) == FALSE)

    toSmooth = data.frame("x" = d[idx,1],"y" = d[idx,a])

    lModel <- loess(y ~ x, data = toSmooth , span = 0.25)
    sm <- predict(lModel)

    d[,a_smoothed]  = rep(NA,nrow(d))
    d[idx,a_smoothed] = sm;


}


names(d) = do.call('rbind',lapply(names(d),function(x){str_replace_all(x,"\\.","-")}))
write.table(d,file="../src/curves_smoothed.csv",quote = FALSE, sep = ",", dec = ".", na = "", row.names = FALSE)
