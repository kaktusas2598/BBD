start_byte = 96,
stop_byte  = A9,

module.exports = {
	var RS485_Message = {
			
			address = FF,
			len = 8,//Can be more. //TODO: test with board
			datalen = 1 // turn relay
			//message = new Buffer
			//data,
			init = function(addr, datalen, data){
				data = Buffer.alloc(datalen)
				msg = Buffer.from([start_byte,address,datalen,data,stop_byte],'utf8');//TODO: CRC16 before stop byte
			},
			crs16 = function(){
				//TODO:calculate
			

	};
}
//message = Buffer.alloc(len)
//msg = Buffer.from([start_byte,address,datalen,data,stop_byte],'utf8');//TODO: CRC16 before stop byte

