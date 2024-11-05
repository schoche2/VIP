CREATE TABLE jobs (
      job_id INT AUTO_INCREMENT PRIMARY KEY,
      name VARCHAR(50) NOT NULL,
      type VARCHAR(50),
      xpos DECIMAL(10),
      ypos DECIMAL(10)
);