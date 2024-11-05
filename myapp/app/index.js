const express = require('express');
const path = require('path');
const mysql = require('mysql2');

const app = express();
app.use(express.json());

// comment in for usage in DOCKER !!!

const pool = mysql.createPool({
  host: process.env.DB_HOST,
  user: process.env.DB_USER,
  password: process.env.DB_PASSWORD,
  database: process.env.DB_NAME,
  waitForConnections: true,
  connectionLimit: 10,
  queueLimit: 0,
});


// comment in for usage in INTELLIJ !!!
/*
const pool = mysql.createPool({
  host: "localhost",
  user: "root",
  password: "root",
  database: "VIP",
  waitForConnections: true,
  connectionLimit: 10,
  queueLimit: 0,
});

 */

// Serve static HTML file for the root route
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

// Route to add an entry
app.post('/entry', (req, res) => {
  const { name } = req.body;
  console.log("In /entry - Name: " + name);
  const query = "INSERT INTO jobs (name) VALUES (?)";

  pool.query(query, [name], (err, result) => {
    if (err) {
      console.error("Error adding entry:", err); // Log the detailed error
      return res.status(500).send("Error adding entry to the database.");
    }
    res.status(201).send("Entry added!");
  });
});



// Route to get all entries
app.get('/entries', (req, res) => {
  const query = "SELECT * FROM jobs";
  pool.query(query, (err, results) => {
    if (err) return res.status(500).send(err);
    res.json(results);
  });
});

app.listen(3000, () => {
  console.log("Server running on port 3000");
});


