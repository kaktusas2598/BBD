var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/', function(req, res, next) {

    //Send ports to view
    req.app.serialport.list(function (err, ports) {
        res.render('index', { title: 'Control Studio',ports:ports });
    });

    //var socket = io();            // socket.io instance. Connects back to the server

    //const requestBuffer = new Buffer("96150101A9","hex");
    //socket.emit('message', "\x96\x15\x01\x01\xA9");       // send the data to the client


    // when new data comes in the websocket, read it:
    //socket.on('message', readData);

    //res.render('index', { title: 'Express',ports:req.app.get('posts')[>['/dev/ttyUSB0','/dev/null']<] });
});
router.post('/' , function(req, res){
    console.log('POST');
});
//function setup() {
//}

//function readData (data) {
//console.log(data);
//}


module.exports = router;

/*module.exports = function(io) {
//var app = require('express');
//var router = app.Router();

io.on('connection', function(socket) { 
console.log('works');
});

return router;
}*/
