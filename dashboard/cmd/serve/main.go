// Command serve hosts the dashboard's static files for local use.
package main

import (
	"flag"
	"log"
	"net/http"
)

func main() {
	addr := flag.String("addr", "127.0.0.1:8080", "address to listen on")
	dir := flag.String("dir", "web", "directory to serve")
	flag.Parse()

	log.Printf("dashboard at http://%s", *addr)
	log.Fatal(http.ListenAndServe(*addr, http.FileServer(http.Dir(*dir))))
}
