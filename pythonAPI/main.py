import mysql.connector


def fetch_all_entries(ip, port, user, password, database, table_name):
    # Connect to the database
    conn = mysql.connector.connect(
        host=ip,
        port=port,
        user=user,
        password=password,
        database=database
    )
    cursor = conn.cursor()

    # Execute a query to select all entries from the table
    cursor.execute(f"SELECT * FROM {table_name}")
    entries = cursor.fetchall()

    # Close the connection
    conn.close()

    return entries


def fetch_lowest_id_row(ip, port, user, password, database, table_name):
    # Connect to the database
    conn = mysql.connector.connect(
        host=ip,
        port=port,
        user=user,
        password=password,
        database=database
    )
    cursor = conn.cursor()

    # Execute a query to select the row with the lowest id
    query = f"SELECT * FROM {table_name} ORDER BY job_id ASC LIMIT 1"
    cursor.execute(query)
    row = cursor.fetchone()

    # Close the connection
    conn.close()

    return row


def delete_row_by_id(ip, port, user, password, database, table_name, row_id):
    # Connect to the database
    conn = mysql.connector.connect(
        host=ip,
        port=port,
        user=user,
        password=password,
        database=database
    )
    cursor = conn.cursor()

    try:
        # Execute the delete statement for the specified row id
        delete_query = f"DELETE FROM {table_name} WHERE job_id = %s"
        cursor.execute(delete_query, (row_id,))
        conn.commit()

        # Check if a row was actually deleted
        if cursor.rowcount > 0:
            print(f"Row with id {row_id} was deleted successfully.")
        else:
            print(f"No row found with id {row_id}.")

    finally:
        # Close the connection
        conn.close()


if __name__ == '__main__':
    print('Hello VIP')

    ip = '127.0.0.1'
    port = 3306
    user = 'root'
    password = 'root'
    database = 'VIP'
    table_name = 'jobs'

    '''
    entries = fetch_all_entries(ip, port, user, password, database, table_name)
    for entry in entries:
        print(entry)
    '''

    '''
    lowest_id_row = fetch_lowest_id_row(ip, port, user, password, database, table_name)
    print(lowest_id_row)

    delete_row_by_id(ip, port, user, password, database, table_name, lowest_id_row[0])

    lowest_id_row = fetch_lowest_id_row(ip, port, user, password, database, table_name)
    print(lowest_id_row)
    '''