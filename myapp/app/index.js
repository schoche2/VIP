const express = require('express');
const path = require('path');
const mysql = require('mysql2');

const app = express();
app.use(express.json());
// needed to use .css file
app.use(express.static(path.join(__dirname, 'public')));

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
  const name = req.body.name;
  const type = req.body.type;
  const xpos = req.body.xpos;
  const ypos = req.body.ypos;
  console.log(req.body);
  const sql = "INSERT INTO jobs VALUES(null, ?, ?, ?, ?) ";
  pool.query(sql, [ name, type, xpos, ypos], (err, ) => {
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

app.delete('/entry/:id', (req, res) => {
  const id = req.params.id;
  const sql = "DELETE FROM jobs WHERE job_id = ?";

  pool.query(sql, [id], (err, result) => {
    if (err) {
      console.error("Error deleting entry:", err); // Log the detailed error
      return res.status(500).send("Error deleting entry from the database.");
    }

    if (result.affectedRows === 0) {
      return res.status(404).send("Entry not found.");
    }

    res.status(200).send("Entry deleted successfully!");
  });
});

app.listen(3000, () => {
  console.log("Server running on port 3000");
});


