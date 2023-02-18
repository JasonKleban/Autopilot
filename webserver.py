from state import state
import utime

async def web_page(reader, writer):
    request = yield from reader.readline()
    
    if (request.startswith("GET / ")):
        state.count += 1
        
        now = utime.time()
        
        upseconds = now - state.boot
        
        html = """<html><head><title>Status</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,"><style>body { text-align: center; font-family: "Trebuchet MS", Arial;}</style>
</head><body><h1>ESP with BME680</h1>
<table>
<tr><td>Uptime</td><td><span>""" + str(upseconds) + """s """ + str(int(round(state.cycleProgress * 100, 0))) + """%</span></td></tr>
<tr><td>Temp. Fahrenheit</td><td><span>""" + state.temp + """</span></td></tr>
<tr><td>Humidity</td><td><span>""" + state.hum + """</span></td></tr>
<tr><td>Status</td><td><span>""" + str(state.indicators) + """ / """ + state.status + """</span></td></tr></body></html>"""

        writer.write("""HTTP/1.1 200 OK
Content-Type: text/html;
Content-Length: """ + str(len(html)) + """

""" + html)
    else:
        print(request + " -> NOT FOUND")
        writer.write("""HTTP/1.1 404 NOT FOUND

""")
        
    await writer.drain()

    reader.close()
    writer.close()
    await writer.wait_closed()
