data.all = read.csv("data.csv", header=T, sep=",")
attach(data.all)

makeplots = function (myplot.problem) {

    par(lwd=1.5)
    par(mfrow=c(2,3))
    colors = 2 + (1:5)

    line_random = data.all[problem==myplot.problem & algorithm=="random", "quality"]
    line_greedy = data.all[problem==myplot.problem & algorithm=="greedy", "quality"]
    line_optimal = data.all[problem==myplot.problem & algorithm=="optimal", "quality"]

    for( myplot.base in c("random", "greedy") ) {
        yrange = range(data.all[problem==myplot.problem & base==myplot.base & algorithm!="random", "quality"], na.rm=TRUE)
        for( myplot.algorithm in c("uct", "ao3", "ao4") ) {
            if( myplot.algorithm == "uct" ) {
                parameters = c(0,-2,-5,-10,-20) #seq(0,-12,by=-3)
            } else {
                parameters = c(0,.2,.4,.6,.8,1) #seq(0,-12,by=-3)
            }
    
            for( i in 1:5 ) {
                # extract data
                p = parameters[i]
                data.plot = data.all[problem==myplot.problem &
                                     algorithm==myplot.algorithm &
                                     base==myplot.base &
                                     par==p,
                                     c("width", "quality", "time")]
                data.plot = data.plot[order(data.plot$width),]
                par(col=colors[i])
                if( i == 1 ) {
                    plot(log(data.plot$width), data.plot$quality, type='l', ylim=yrange, frame.plot=FALSE, ylab="quality", xlab="width")
                } else {
                    lines(log(data.plot$width), data.plot$quality, type='l')
                }
            }
    
            # horizontal lines
            par(col=1, lty=2, lwd=0.5)
            abline(h=line_random)
            #text(x=190,y=.25+line_random, labels=c("random"))
            abline(h=line_greedy)
            #text(x=190,y=.25+line_greedy, labels=c("greedy"))
            abline(h=line_optimal)
            #text(x=190,y=.25+line_optimal, labels=c("optimal"))
            par(lty=1, lwd=1.5)
    
            # title
            title(paste(myplot.problem, ":", myplot.algorithm, "w/", myplot.base))
        }
    }

    return(0)
}




# legend & title

#par(lty=1,lwd=1)
#legend(locator(1),paste("C=",parameters,sep=""), col=colors, lty=rep(1,3), horiz=TRUE)

